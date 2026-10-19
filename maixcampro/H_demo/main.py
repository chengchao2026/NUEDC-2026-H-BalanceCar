import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from maix import camera, display, image, nn, app, time, sys, uart, pinmap, err, network

from ball_position import (
    AdaptiveAlphaBetaFilter,
    axis_point,
    axis_velocity,
    position_from_pixel,
    validate_calibration,
)

# 漏检容忍窗口：检测不到球的这段时间内，用滤波器的速度短时预测位置继续
# 发 valid=1，避免偶尔漏检 1~2 帧就让STM32判定丢球停机。必须小于STM32那边
# 的 target_fresh_timeout_ms(150ms)，否则STM32会先按超时处理。
VISION_HOLD_MS = 120

# MaixCam自建AP热点，手机/电脑直接连它即可看画面，不再依赖第三方路由器
# 或手机热点中转。AP模式下MaixCam本身不再联外网。
WIFI_AP_SSID = "MaixCam-Ball"
WIFI_AP_PASSWORD = "12345678"
WIFI_AP_IP = "192.168.66.1"


def connect_wifi():
    """Best-effort AP-mode startup. Core ball tracking only needs the camera
    and UART, so a failure here must not block that -- only WebRTC remote
    viewing depends on this succeeding. Failures are logged and swallowed
    instead of raised."""
    wifi = network.wifi.Wifi()
    print("[WiFi] 正在启动热点:", WIFI_AP_SSID)
    try:
        result = wifi.start_ap(WIFI_AP_SSID, WIFI_AP_PASSWORD, ip=WIFI_AP_IP)
        err.check_raise(result, "热点启动失败")
        print("[WiFi] 热点已启动，IP:", WIFI_AP_IP, " 密码:", WIFI_AP_PASSWORD)
        return WIFI_AP_IP
    except Exception as exc:
        print("[WiFi] 热点启动失败，本次跳过远程画面，仅用摄像头+UART运行:", exc)
        return None


def init_uart():
    # Same wiring as ball_balance.py: UART0 on A16(TX)/A17(RX), /dev/ttyS0.
    # NOTE: UART0/A16-A17 can be claimed internally by MaixPy's "Maix Comm
    # Protocol" listener whenever camera/display are used (this is why the
    # original H_demo script avoided it and used UART1 instead). Physical
    # wiring here is fixed to A16/A17, so if the STM32 side stops receiving
    # frames after switching to this build, that internal listener is the
    # first thing to check/disable.
    pin_function = {"A16": "UART0_TX", "A17": "UART0_RX"}
    device = "/dev/ttyS0"

    for pin, func in pin_function.items():
        err.check_raise(pinmap.set_pin_function(pin, func),
                         "Failed to configure pin %s as %s" % (pin, func))

    return uart.UART(device, 115200)

# 输入打印. 关掉后提升帧率
DEBUG_LOG=False

def print_debug(s):
    if DEBUG_LOG:
        print(s)

# 位置传感器更重视“采集到结果”的延迟，而不是最高吞吐帧率。
# 低延迟模式下 dual_buff=False，每次检测结果都对应本次传入的图像。
# 只有在测试最高检测帧率时，才建议把该开关改为 False。
LOW_LATENCY_MODE = True

# RTSP流
USE_RTSP=False

# HTTP JPEG流
USE_JPEG=False

# WEBRTC流
USE_WEBRTC=True

# 使能畸变校准
LENS_CORR_ENABLE=False

# 畸变校准强度
LENS_CORR_STRENGTH=0.6

# 钢珠位置两点标定。固定摄像头后，把钢珠分别放在两个已知刻度上，
# 将检测框中心像素和实际刻度填写到这里。起点为负方向，终点为正方向。
# 以下像素值仅为初始示例，使用前必须实测。
AXIS_START_PX = (40, 112)
AXIS_END_PX = (280, 112)
AXIS_START_CM = 0       # cm
AXIS_END_CM = 25          # cm
DETECTION_CONFIDENCE = 0.50

# 只用MaixCAM Pro这一个型号的模型，模型文件直接放在跟main.py同一级目录下
# (不用models子文件夹)，跟app.yaml里的files列表保持一致，避免MaixVision
# 打包工具处理嵌套目录时报错。maixcam2的模型文件已经不在了，不支持的设备
# 直接报错，避免误加载模型。
MAIXCAM_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "yolo26_all_maixcam.mud",
)

def model_path_for_device(device_name):
    """Return the checked-in YOLO26 model matching a Maix device name."""
    normalized_name = device_name.strip().lower()
    if normalized_name in ("maixcam", "maixcam-pro", "maixcam_pro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


model = model_path_for_device(sys.device_name())

detector = nn.YOLO26(
    model=model,
    dual_buff=not LOW_LATENCY_MODE,
)

# 注意：不要在这里用探测器输入宽高自动覆盖 AXIS_START_PX/AXIS_END_PX，
# 那样会让上面的实测标定形同虚设。摆杆两端的像素坐标必须实测后填在上面。

# 固定ROI：摄像头装好后管子在画面里的位置不再变化，检测框中心不落在这个
# 矩形内的一律当误检丢弃（背景、小车结构等）。margin留够球的直径+抖动余量。
ROI_MARGIN_PX = max(20, detector.input_height() // 8)
ROI_X0 = min(AXIS_START_PX[0], AXIS_END_PX[0]) - ROI_MARGIN_PX
ROI_X1 = max(AXIS_START_PX[0], AXIS_END_PX[0]) + ROI_MARGIN_PX
ROI_Y0 = min(AXIS_START_PX[1], AXIS_END_PX[1]) - ROI_MARGIN_PX
ROI_Y1 = max(AXIS_START_PX[1], AXIS_END_PX[1]) + ROI_MARGIN_PX


def in_roi(cx, cy):
    return ROI_X0 <= cx <= ROI_X1 and ROI_Y0 <= cy <= ROI_Y1


cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()
position_filter = AdaptiveAlphaBetaFilter()
serial = init_uart()
print_debug("[DEBUG] UART opened")

WEB_PORT = 8000
SIGNALING_PORT = 8001
webrtc_server = None

WEBRTC_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
  <title>MaixCAM 实时画面</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #15171a; color: #fff; }
    main { width: min(920px, 100%); margin: auto; padding: 16px; }
    h1 { margin: 4px 0 14px; font-size: 22px; }
    .video-wrap { position: relative; overflow: hidden; border-radius: 14px; background: #000; }
    video { display: block; width: 100%; max-height: 70vh; background: #000; }
    .badge { position: absolute; left: 10px; bottom: 10px; padding: 5px 9px;
             border-radius: 8px; background: #000a; font-size: 13px; }
    .controls { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 14px; }
    button { min-height: 48px; border: 0; border-radius: 12px; font-size: 17px;
             font-weight: 700; color: white; }
    button:disabled { opacity: .42; }
    #start { background: #16894c; }
    #stop { background: #c93636; }
    .status { display: flex; justify-content: space-between; gap: 12px; margin-top: 12px;
              padding: 10px 12px; border-radius: 10px; background: #ffffff0d; }
    #download { display: none; margin-top: 12px; padding: 12px; border-radius: 10px;
                text-align: center; background: #1769aa; color: #fff; text-decoration: none; }
    .hint { color: #b9bec7; font-size: 13px; line-height: 1.5; }
  </style>
</head>
<body>
<main>
  <h1>MaixCAM 实时画面</h1>
  <div class="video-wrap">
    <video id="video" autoplay playsinline muted></video>
    <div class="badge"><span id="resolution">等待视频</span></div>
  </div>
  <div class="controls">
    <button id="start" disabled>开始录制</button>
    <button id="stop" disabled>停止并保存</button>
  </div>
  <div class="status">
    <span id="state">正在连接……</span>
    <span id="timer">00:00</span>
  </div>
  <a id="download">下载录制视频</a>
  <p class="hint">视频由当前浏览器录制并保存在手机或电脑中，不写入 MaixCAM 的 TF 卡。</p>
</main>
<script>
  const video = document.getElementById('video');
  const startButton = document.getElementById('start');
  const stopButton = document.getElementById('stop');
  const stateText = document.getElementById('state');
  const timerText = document.getElementById('timer');
  const resolutionText = document.getElementById('resolution');
  const downloadLink = document.getElementById('download');

  let peerConnection = null;
  let mediaRecorder = null;
  let chunks = [];
  let timerHandle = null;
  let recordStartedAt = 0;
  let lastDownloadUrl = null;
  let websocket = null;
  let reconnectHandle = null;

  const clientId = Array.from(crypto.getRandomValues(new Uint8Array(10)))
    .map(value => (value % 36).toString(36)).join('');
  const wsUrl = `ws://${location.hostname}:8001/${clientId}`;

  function connectSignaling() {
    clearTimeout(reconnectHandle);
    stateText.textContent = '正在连接……';
    const socket = new WebSocket(wsUrl);
    websocket = socket;

    socket.onopen = () => {
      if (websocket !== socket) return;
      stateText.textContent = '信令已连接，等待视频';
      socket.send(JSON.stringify({id: 'server', type: 'request'}));
    };

    socket.onmessage = async event => {
      if (websocket !== socket || typeof event.data !== 'string') return;
      const message = JSON.parse(event.data);
      if (message.type === 'offer') await handleOffer(message);
    };

    socket.onerror = () => {
      if (websocket === socket) stateText.textContent = '连接失败，正在重试……';
    };
    socket.onclose = () => {
      if (websocket !== socket) return;
      stateText.textContent = '连接已断开，正在重试……';
      startButton.disabled = true;
      if (mediaRecorder && mediaRecorder.state === 'recording') mediaRecorder.stop();
      reconnectHandle = setTimeout(connectSignaling, 1500);
    };
  }

  function createPeerConnection() {
    const pc = new RTCPeerConnection({
      bundlePolicy: 'max-bundle',
      iceServers: [{urls: ['stun:stun.l.google.com:19302']}]
    });

    pc.ontrack = event => {
      video.srcObject = event.streams[0];
      video.play().catch(() => {});
      startButton.disabled = false;
      stateText.textContent = '视频已连接';
    };

    pc.onconnectionstatechange = () => {
      if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)) {
        stateText.textContent = `WebRTC ${pc.connectionState}`;
        startButton.disabled = true;
      }
    };
    return pc;
  }

  async function waitForIce() {
    if (peerConnection.iceGatheringState === 'complete') return;
    await new Promise(resolve => {
      const listener = () => {
        if (peerConnection.iceGatheringState === 'complete') {
          peerConnection.removeEventListener('icegatheringstatechange', listener);
          resolve();
        }
      };
      peerConnection.addEventListener('icegatheringstatechange', listener);
    });
  }

  async function handleOffer(offer) {
    if (peerConnection) peerConnection.close();
    peerConnection = createPeerConnection();
    await peerConnection.setRemoteDescription(offer);
    await peerConnection.setLocalDescription(await peerConnection.createAnswer());
    await waitForIce();
    if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
    websocket.send(JSON.stringify({
      id: 'server',
      type: peerConnection.localDescription.type,
      sdp: peerConnection.localDescription.sdp
    }));
  }

  video.addEventListener('loadedmetadata', () => {
    resolutionText.textContent = `${video.videoWidth} × ${video.videoHeight}`;
  });

  function chooseMimeType() {
    const types = [
      'video/webm;codecs=vp8,opus',
      'video/webm;codecs=vp8',
      'video/webm',
      'video/mp4'
    ];
    return types.find(type => MediaRecorder.isTypeSupported(type)) || '';
  }

  function updateTimer() {
    const seconds = Math.floor((Date.now() - recordStartedAt) / 1000);
    timerText.textContent = `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  }

  startButton.onclick = () => {
    if (!video.srcObject || typeof MediaRecorder === 'undefined') {
      stateText.textContent = '当前浏览器不支持视频录制';
      return;
    }

    chunks = [];
    downloadLink.style.display = 'none';
    if (lastDownloadUrl) URL.revokeObjectURL(lastDownloadUrl);

    const mimeType = chooseMimeType();
    mediaRecorder = new MediaRecorder(video.srcObject, mimeType ? {mimeType} : undefined);
    mediaRecorder.ondataavailable = event => {
      if (event.data && event.data.size) chunks.push(event.data);
    };
    mediaRecorder.onerror = event => {
      stateText.textContent = `录制失败：${event.error?.message || '未知错误'}`;
    };
    mediaRecorder.onstop = () => {
      clearInterval(timerHandle);
      const finalType = mediaRecorder.mimeType || mimeType || 'video/webm';
      const blob = new Blob(chunks, {type: finalType});
      lastDownloadUrl = URL.createObjectURL(blob);
      const extension = finalType.includes('mp4') ? 'mp4' : 'webm';
      const stamp = new Date().toISOString().replace(/[:.]/g, '-');
      downloadLink.href = lastDownloadUrl;
      downloadLink.download = `maixcam-${stamp}.${extension}`;
      downloadLink.textContent = `下载录制视频（${(blob.size / 1048576).toFixed(1)} MB）`;
      downloadLink.style.display = 'block';
      stateText.textContent = '录制完成，请点击下载';
      startButton.disabled = false;
      stopButton.disabled = true;
    };

    mediaRecorder.start(1000);
    recordStartedAt = Date.now();
    updateTimer();
    timerHandle = setInterval(updateTimer, 500);
    stateText.textContent = '正在录制';
    startButton.disabled = true;
    stopButton.disabled = false;
  };

  stopButton.onclick = () => {
    if (mediaRecorder && mediaRecorder.state === 'recording') mediaRecorder.stop();
  };

  connectSignaling();
</script>
</body>
</html>
"""


class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True


class WebRtcPageHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return

        body = WEBRTC_PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        pass


def start_webrtc_page():
    httpd = ReusableHTTPServer(("0.0.0.0", WEB_PORT), WebRtcPageHandler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


if USE_RTSP:
    from maix import rtsp
    cam2 = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    server = rtsp.Rtsp()
    server.bind_camera(cam2)
    server.start()
    print(server.get_url())

if USE_JPEG:
    from maix import http
    jpeg_server = http.JpegStreamer()
    jpeg_server.start()

if USE_WEBRTC:
    from maix import webrtc
    cam2 = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    wifi_ip = connect_wifi()
    if wifi_ip:
        try:
            # AP接口完全就绪后，再按MaixPy官方示例在主线程启动WebRTC。
            time.sleep_ms(1000)
            webrtc_server = webrtc.WebRTC(
                signaling_port=SIGNALING_PORT,
                http_server=False,
            )
            webrtc_server.bind_camera(cam2)
            webrtc_server.start()
            start_webrtc_page()
            print("[WebRTC] 已启动，信令端口:", SIGNALING_PORT)
            print("浏览器打开: http://%s:%d" % (wifi_ip, WEB_PORT))
        except Exception as exc:
            webrtc_server = None
            print("[WebRTC] 启动失败:", exc)

validate_calibration(
    AXIS_START_PX,
    AXIS_END_PX,
    AXIS_START_CM,
    AXIS_END_CM,
    detector.input_width(),
    detector.input_height(),
)

print(detector.input_width(), detector.input_height())

axis_color = image.Color.from_rgb(0, 220, 255)
zero_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)


def draw_calibration_axis(img):
    img.draw_line(
        AXIS_START_PX[0], AXIS_START_PX[1],
        AXIS_END_PX[0], AXIS_END_PX[1],
        axis_color, 2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)
    zero_ratio = (0.0 - AXIS_START_CM) / (AXIS_END_CM - AXIS_START_CM)
    zero_x, zero_y = axis_point(zero_ratio, AXIS_START_PX, AXIS_END_PX)
    img.draw_cross(int(zero_x), int(zero_y), zero_color, 9, 2)

last_ms = time.ticks_ms()
loop_ms = 1000
while not app.need_exit():
    loop_ms = time.ticks_ms() - last_ms
    print_debug("loop cost " + str(loop_ms) + "ms")
    last_ms = time.ticks_ms()

    try:
        img = cam.read()
    except Exception as exc:
        # Documented MaixPy behavior: an occasional read timeout happens when
        # the buffer isn't refreshed in time. Retry instead of crashing the
        # whole app (a hard crash here was leaving the ISP/multimedia driver
        # in a bad state for the next launch).
        print_debug("[WARN] camera read failed, retrying: " + str(exc))
        time.sleep_ms(10)
        continue
    t = time.ticks_ms()

    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)
    print_debug("lens_corr cost " + str(time.ticks_ms() - t) + "ms")

    t = time.ticks_ms()
    objs = detector.detect(img, conf_th=DETECTION_CONFIDENCE, iou_th=0.45)
    print_debug("detect cost " + str(time.ticks_ms() - t) + "ms")

    t = time.ticks_ms()
    now_ms = time.ticks_ms()
    draw_calibration_axis(img)

    # 固定ROI过滤：检测框中心不在管子附近的直接丢弃（背景/小车结构误检）。
    objs_in_roi = [
        obj for obj in objs
        if in_roi(obj.x + obj.w * 0.5, obj.y + obj.h * 0.5)
    ]
    # 模型只有 steel_ball 一个类别。如果一帧出现多个框，只选择置信度最高的框，
    # 避免多个误检框同时干扰位置滤波器。
    ball = max(objs_in_roi, key=lambda obj: obj.score) if objs_in_roi else None

    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        # 时序合理性门控：这一帧的位置离滤波器预测太远，当误检丢弃，走下面
        # 的"丢球"分支（会用短时预测撑住，不会立刻让STM32停机）。
        if not position_filter.is_plausible(raw_x, raw_y, now_ms):
            ball = None

    if ball is not None:
        filtered_x, filtered_y = position_filter.update(raw_x, raw_y, now_ms)
        position_cm, axis_ratio, _axis_distance = position_from_pixel(
            (filtered_x, filtered_y),
            AXIS_START_PX,
            AXIS_END_PX,
            AXIS_START_CM,
            AXIS_END_CM,
        )
        projected_x, projected_y = axis_point(
            axis_ratio, AXIS_START_PX, AXIS_END_PX
        )

        img.draw_rect(ball.x, ball.y, ball.w, ball.h, color=image.COLOR_GREEN, thickness=2)

        if DEBUG_LOG:
            img.draw_cross(
                int(filtered_x), int(filtered_y), image.COLOR_GREEN, 12, 2
            )
            img.draw_cross(
                int(projected_x), int(projected_y), zero_color, 7, 2
            )
            msg = (
                f"{detector.labels[ball.class_id]}: {ball.score:.2f} "
                f"px=({filtered_x:.1f},{filtered_y:.1f})"
            )
            img.draw_string(ball.x, ball.y, msg, color=image.COLOR_RED)
            img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)
        img.draw_string(
            8, 5, f"BALL {position_cm:+.2f} cm",
            color=image.COLOR_WHITE, scale=1.4, thickness=2,
        )

        # 发给STM32: $C,<valid>,<pos_cm*100>,<vel_cms*100>\n
        # 定点数(乘100取整)传输，MCU端不用解析浮点数。
        vel_cms = axis_velocity(
            position_filter.vx, position_filter.vy,
            AXIS_START_PX, AXIS_END_PX,
            AXIS_START_CM, AXIS_END_CM,
        )
        tx_data = "$C,1,{},{}\n".format(
            int(round(position_cm * 100)), int(round(vel_cms * 100))
        )
        serial.write_str(tx_data)
    else:
        missing_ms = (
            now_ms - position_filter.last_ms
            if position_filter.last_ms is not None else None
        )
        predicted = position_filter.predict(now_ms)
        position_filter.mark_missing(now_ms)

        if predicted is not None and missing_ms is not None and missing_ms <= VISION_HOLD_MS:
            pred_x, pred_y = predicted
            position_cm, _axis_ratio, _axis_distance = position_from_pixel(
                (pred_x, pred_y),
                AXIS_START_PX,
                AXIS_END_PX,
                AXIS_START_CM,
                AXIS_END_CM,
            )
            vel_cms = axis_velocity(
                position_filter.vx, position_filter.vy,
                AXIS_START_PX, AXIS_END_PX,
                AXIS_START_CM, AXIS_END_CM,
            )

            img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)
            img.draw_string(
                8, 5, f"BALL {position_cm:+.2f} cm (predicted)",
                color=zero_color, scale=1.4, thickness=2,
            )

            tx_data = "$C,1,{},{}\n".format(
                int(round(position_cm * 100)), int(round(vel_cms * 100))
            )
            serial.write_str(tx_data)
        else:
            img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)
            img.draw_string(
                8, 5, "BALL LOST",
                color=image.COLOR_RED, scale=1.4, thickness=2,
            )

            # 短时预测窗口也过了，才真正判定丢球，发 valid=0 让STM32停机。
            serial.write_str("$C,0,0,0\n")

    fps_str = "FPS:" + str(0 if loop_ms == 0 else 1000//loop_ms)
    img.draw_string(
        img.width() - image.string_size(fps_str, scale=1.4, thickness=2).width(), 5, fps_str,
        color=image.COLOR_GREEN, scale=1.4, thickness=2,
    )
    print_debug("draw and get position cost " + str(time.ticks_ms() - t) + "ms")

    if USE_JPEG:
        jpeg_server.write(img)
    disp.show(img)
