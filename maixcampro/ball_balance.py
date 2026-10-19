from maix import app, camera, display, image, time, uart, pinmap, err

# Tune these values on the MaixCam Pro before connecting the motor.
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
GRAY_THRESHOLD = (0, 80)  # Black ball on a brighter background.
MIN_PIXELS = 80
MIN_AREA = 80
MAX_AREA = 6000
MIN_ASPECT_RATIO = 0.55
MAX_ASPECT_RATIO = 1.80
ROI = (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
# Default MaixCam Pro UART0 pin mapping. Change both pin names and device if wired differently.
UART_TX_PIN = "A16"
UART_RX_PIN = "A17"
UART_DEVICE = "/dev/ttyS0"
UART_BAUDRATE = 115200
SEND_INTERVAL_MS = 30


def select_ball(blobs):
    best_blob = None
    best_score = -1

    for blob in blobs:
        width = blob.w()
        height = blob.h()
        if width == 0 or height == 0:
            continue

        aspect_ratio = float(width) / float(height)
        area = blob.area()
        if (area < MIN_AREA or area > MAX_AREA or
                aspect_ratio < MIN_ASPECT_RATIO or
                aspect_ratio > MAX_ASPECT_RATIO):
            continue

        # Prefer the largest near-circular connected region.
        score = blob.pixels() - abs(width - height) * 10
        if score > best_score:
            best_blob = blob
            best_score = score

    return best_blob


def main():
    err.check_raise(pinmap.set_pin_function(UART_TX_PIN, "UART0_TX"),
                    "Failed to configure MaixCam Pro UART TX pin")
    err.check_raise(pinmap.set_pin_function(UART_RX_PIN, "UART0_RX"),
                    "Failed to configure MaixCam Pro UART RX pin")

    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT, image.Format.FMT_GRAYSCALE)
    disp = display.Display()
    serial = uart.UART(UART_DEVICE, UART_BAUDRATE)
    last_send_ms = 0

    while not app.need_exit():
        img = cam.read()
        blobs = img.find_blobs([GRAY_THRESHOLD], roi=ROI,
                                pixels_threshold=MIN_PIXELS,
                                area_threshold=MIN_AREA)
        ball = select_ball(blobs)

        if ball is not None:
            img.draw_rectangle(ball.x(), ball.y(), ball.w(), ball.h(),
                               color=image.COLOR_WHITE)
            img.draw_cross(ball.cx(), ball.cy(), color=image.COLOR_WHITE)
        disp.show(img)

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
            if ball is None:
                serial.write_str("$B,0,0\n")
            else:
                serial.write_str("$B,1,{}\n".format(ball.cx()))
            last_send_ms = now_ms


if __name__ == "__main__":
    main()
