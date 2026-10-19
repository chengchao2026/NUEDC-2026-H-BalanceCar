import csv
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


@dataclass(frozen=True)
class Parameter:
    key: str
    label: str
    scale: int
    unit: str


PARAMETERS = [
    Parameter("OKP", "Outer position Kp", 1000, "deg/cm"),
    Parameter("OKI", "Outer position Ki", 1000, "deg/(cm*s)"),
    Parameter("OKD", "Outer position Kd", 1000, "deg*s/cm"),
    Parameter("IKP", "Inner angle Kp", 1000, "1/s"),
    Parameter("IKI", "Inner angle Ki", 1000, "1/s2"),
    Parameter("IKD", "Inner angle Kd", 1000, ""),
    Parameter("ALIM", "Target angle limit", 100, "deg"),
    Parameter("PWM", "PWM speed limit", 100, "deg/s"),
]

PARAM_BY_KEY = {item.key: item for item in PARAMETERS}
CSV_FIELDS = [
    "host_time_s",
    "mcu_tick_ms",
    "valid",
    "raw_position_cm",
    "estimated_position_cm",
    "estimated_velocity_cms",
    "pwm_command_deg_s",
    "angle_error_deg",
    "target_angle_deg",
    "command_angle_deg",
    "motor_position_steps",
    "state",
]


class SerialWorker:
    def __init__(self, receive_queue):
        self.receive_queue = receive_queue
        self.port = None
        self.thread = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self):
        return self.port is not None and self.port.is_open

    def connect(self, port_name, baudrate):
        self.disconnect()
        self.port = serial.Serial(port_name, baudrate, timeout=0.1)
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.port is not None:
            try:
                self.port.close()
            except serial.SerialException:
                pass
        self.port = None

    def send(self, line):
        if not self.connected:
            raise RuntimeError("Serial port is not connected")
        payload = (line.rstrip("\r\n") + "\n").encode("ascii")
        with self.write_lock:
            self.port.write(payload)

    def _reader(self):
        while not self.stop_event.is_set() and self.connected:
            try:
                raw = self.port.readline()
                if raw:
                    self.receive_queue.put(raw.decode("ascii", errors="replace").strip())
            except (serial.SerialException, OSError) as exc:
                self.receive_queue.put(("SERIAL_ERROR", str(exc)))
                break


class TelemetryPlot(ttk.Frame):
    COLORS = {
        "raw_position": "#8b949e",
        "estimated_position": "#1f6feb",
        "target_position": "#2da44e",
        "estimated_velocity": "#d29922",
        "pwm_command": "#8250df",
        "angle_error": "#bf8700",
        "target_angle": "#cf222e",
        "command_angle": "#0969da",
    }

    def __init__(self, parent):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, background="#ffffff", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.samples = deque(maxlen=400)

    def add_sample(self, sample):
        self.samples.append(sample)
        self.redraw()

    def clear(self):
        self.samples.clear()
        self.redraw()

    def redraw(self):
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 500)
        height = max(canvas.winfo_height(), 360)
        margin_left = 54
        margin_right = 12
        panel_gap = 18
        panel_height = (height - 34 - 3 * panel_gap) / 4.0
        panels = [
            ("Position (cm)", 0.0, 25.0,
             [("raw_position", "raw_position"),
              ("estimated_position", "estimated_position"),
              ("target_position", "target_position")]),
            ("Ball velocity (cm/s)", -15.0, 15.0,
             [("estimated_velocity", "estimated_velocity")]),
            ("Rail angle (deg)", -14.0, 14.0,
             [("target_angle", "target_angle"),
              ("command_angle", "command_angle"),
              ("angle_error", "angle_error")]),
            ("PWM command (deg/s)", -30.0, 30.0,
             [("pwm_command", "pwm_command")]),
        ]

        if self.samples:
            latest_tick = self.samples[-1]["mcu_tick_ms"]
            earliest_tick = max(latest_tick - 15000, self.samples[0]["mcu_tick_ms"])
        else:
            latest_tick = 15000
            earliest_tick = 0
        tick_span = max(latest_tick - earliest_tick, 1)

        for index, (title, y_min, y_max, series) in enumerate(panels):
            top = 18 + index * (panel_height + panel_gap)
            bottom = top + panel_height
            left = margin_left
            right = width - margin_right
            canvas.create_rectangle(left, top, right, bottom, outline="#d0d7de")
            canvas.create_text(8, top + 8, text=title, anchor="w", fill="#24292f")
            canvas.create_text(left - 6, top, text=f"{y_max:g}", anchor="e", fill="#57606a")
            canvas.create_text(left - 6, bottom, text=f"{y_min:g}", anchor="e", fill="#57606a")
            zero_y = bottom - (0.0 - y_min) / (y_max - y_min) * panel_height
            if top <= zero_y <= bottom:
                canvas.create_line(left, zero_y, right, zero_y, fill="#d8dee4", dash=(3, 3))

            legend_x = right - 8
            for key, color_key in reversed(series):
                label = key.replace("_", " ")
                text_id = canvas.create_text(legend_x, top + 10, text=label,
                                             anchor="e", fill=self.COLORS[color_key])
                bounds = canvas.bbox(text_id)
                legend_x = bounds[0] - 14

            for key, color_key in series:
                points = []
                for sample in self.samples:
                    tick = sample["mcu_tick_ms"]
                    if tick < earliest_tick:
                        continue
                    value = max(y_min, min(y_max, sample[key]))
                    x = left + (tick - earliest_tick) / tick_span * (right - left)
                    y = bottom - (value - y_min) / (y_max - y_min) * panel_height
                    points.extend((x, y))
                if len(points) >= 4:
                    canvas.create_line(*points, fill=self.COLORS[color_key], width=2)


class TuningApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ball Balance Tuning Console")
        self.geometry("1180x760")
        self.minsize(960, 620)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.receive_queue = queue.Queue()
        self.worker = SerialWorker(self.receive_queue)
        self.parameter_vars = {}
        self.csv_file = None
        self.csv_writer = None
        self.host_start_time = time.monotonic()

        self._build_ui()
        self.refresh_ports()
        self.after(40, self.process_messages)

    def _build_ui(self):
        toolbar = ttk.Frame(self, padding=8)
        toolbar.pack(fill="x")
        ttk.Label(toolbar, text="Port").pack(side="left")
        self.port_combo = ttk.Combobox(toolbar, width=18, state="readonly")
        self.port_combo.pack(side="left", padx=(6, 4))
        ttk.Button(toolbar, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(toolbar, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Read parameters", command=self.read_parameters).pack(side="left", padx=8)
        ttk.Button(toolbar, text="Send all", command=self.send_all_parameters).pack(side="left")
        ttk.Button(toolbar, text="Clear plots", command=self.clear_plots).pack(side="left", padx=8)
        self.record_button = ttk.Button(toolbar, text="Start CSV", command=self.toggle_recording)
        self.record_button.pack(side="left")
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        pane = ttk.Panedwindow(self, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        parameter_frame = ttk.Frame(pane, padding=6)
        pane.add(parameter_frame, weight=0)
        ttk.Label(parameter_frame, text="Runtime parameters",
                  font=("TkDefaultFont", 10, "bold")).pack(anchor="w", pady=(0, 6))

        scroll_canvas = tk.Canvas(parameter_frame, width=370, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parameter_frame, orient="vertical", command=scroll_canvas.yview)
        form = ttk.Frame(scroll_canvas)
        form.bind("<Configure>", lambda _event: scroll_canvas.configure(
            scrollregion=scroll_canvas.bbox("all")))
        scroll_canvas.create_window((0, 0), window=form, anchor="nw")
        scroll_canvas.configure(yscrollcommand=scrollbar.set)
        scroll_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for row, parameter in enumerate(PARAMETERS):
            ttk.Label(form, text=parameter.label, width=22).grid(
                row=row, column=0, sticky="w", padx=(0, 6), pady=2)
            variable = tk.StringVar(value="")
            self.parameter_vars[parameter.key] = variable
            ttk.Entry(form, textvariable=variable, width=10).grid(row=row, column=1, pady=2)
            ttk.Label(form, text=parameter.unit, width=12).grid(
                row=row, column=2, sticky="w", padx=5)
            ttk.Button(form, text="Set", width=5,
                       command=lambda key=parameter.key: self.send_parameter(key)).grid(
                           row=row, column=3, padx=(2, 0), pady=2)

        plot_frame = ttk.Frame(pane)
        pane.add(plot_frame, weight=1)
        self.plot = TelemetryPlot(plot_frame)
        self.plot.pack(fill="both", expand=True)

        values = ttk.Frame(plot_frame, padding=(4, 6))
        values.pack(fill="x")
        self.live_vars = {name: tk.StringVar(value="--") for name in
                          ("position", "velocity", "pwm_command", "angle", "motor", "state")}
        labels = [
            ("Position", "position", "cm"),
            ("Velocity", "velocity", "cm/s"),
            ("PWM command", "pwm_command", "deg/s"),
            ("Command angle", "angle", "deg"),
            ("Motor", "motor", "steps"),
            ("State", "state", ""),
        ]
        for index, (caption, key, unit) in enumerate(labels):
            ttk.Label(values, text=caption).grid(row=0, column=index, padx=8)
            ttk.Label(values, textvariable=self.live_vars[key],
                      font=("TkDefaultFont", 10, "bold")).grid(row=1, column=index)
            ttk.Label(values, text=unit).grid(row=2, column=index)

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            return
        ports = [item.device for item in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_combo.get() not in ports:
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        if self.worker.connected:
            self.worker.disconnect()
            self.connect_button.configure(text="Connect")
            self.status_var.set("Disconnected")
            return
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial: pip install pyserial")
            return
        port_name = self.port_combo.get()
        if not port_name:
            messagebox.showwarning("Serial port", "Select a serial port first")
            return
        try:
            self.worker.connect(port_name, 115200)
            self.connect_button.configure(text="Disconnect")
            self.status_var.set(f"Connected: {port_name}")
            self.after(150, self.read_parameters)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("Connection failed", str(exc))

    def read_parameters(self):
        self._send("@GET")

    def send_parameter(self, key):
        parameter = PARAM_BY_KEY[key]
        text = self.parameter_vars[key].get().strip()
        try:
            encoded_value = int(round(float(text) * parameter.scale))
        except ValueError:
            messagebox.showwarning("Invalid value", f"{parameter.label}: {text!r}")
            return
        self._send(f"@SET,{key},{encoded_value}")

    def send_all_parameters(self):
        for parameter in PARAMETERS:
            if self.parameter_vars[parameter.key].get().strip():
                self.send_parameter(parameter.key)

    def _send(self, command):
        try:
            self.worker.send(command)
        except RuntimeError:
            self.status_var.set("Connect before sending commands")
        except (serial.SerialException, OSError) as exc:
            self.status_var.set(str(exc))

    def process_messages(self):
        while True:
            try:
                message = self.receive_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(message, tuple) and message[0] == "SERIAL_ERROR":
                self.status_var.set(f"Serial error: {message[1]}")
                self.worker.disconnect()
                self.connect_button.configure(text="Connect")
            elif message.startswith("@P,"):
                self.handle_parameter(message)
            elif message.startswith("@T,"):
                self.handle_telemetry(message)
            elif message.startswith("@OK") or message.startswith("@ERR"):
                self.status_var.set(message)
        self.after(40, self.process_messages)

    def handle_parameter(self, line):
        parts = line.split(",")
        if len(parts) != 3 or parts[1] not in PARAM_BY_KEY:
            return
        parameter = PARAM_BY_KEY[parts[1]]
        try:
            value = int(parts[2]) / parameter.scale
        except ValueError:
            return
        digits = 0 if parameter.scale == 1 else (2 if parameter.scale == 100 else 3)
        self.parameter_vars[parameter.key].set(f"{value:.{digits}f}")

    def handle_telemetry(self, line):
        parts = line.split(",")
        if len(parts) != 12:
            return
        try:
            sample = {
                "mcu_tick_ms": int(parts[1]),
                "valid": int(parts[2]),
                "raw_position": int(parts[3]) / 100.0,
                "estimated_position": int(parts[4]) / 100.0,
                "estimated_velocity": int(parts[5]) / 100.0,
                "pwm_command": int(parts[6]) / 100.0,
                "angle_error": int(parts[7]) / 100.0,
                "target_angle": int(parts[8]) / 100.0,
                "command_angle": int(parts[9]) / 100.0,
                "motor_position_steps": int(parts[10]),
                "state": int(parts[11]),
                "target_position": 12.0,
            }
        except ValueError:
            return

        self.plot.add_sample(sample)
        self.live_vars["position"].set(f"{sample['estimated_position']:.2f}")
        self.live_vars["velocity"].set(f"{sample['estimated_velocity']:.2f}")
        self.live_vars["pwm_command"].set(f"{sample['pwm_command']:.2f}")
        self.live_vars["angle"].set(f"{sample['command_angle']:.2f}")
        self.live_vars["motor"].set(str(sample["motor_position_steps"]))
        self.live_vars["state"].set(str(sample["state"]))

        if self.csv_writer is not None:
            row = {
                "host_time_s": f"{time.monotonic() - self.host_start_time:.3f}",
                "mcu_tick_ms": sample["mcu_tick_ms"],
                "valid": sample["valid"],
                "raw_position_cm": sample["raw_position"],
                "estimated_position_cm": sample["estimated_position"],
                "estimated_velocity_cms": sample["estimated_velocity"],
                "pwm_command_deg_s": sample["pwm_command"],
                "angle_error_deg": sample["angle_error"],
                "target_angle_deg": sample["target_angle"],
                "command_angle_deg": sample["command_angle"],
                "motor_position_steps": sample["motor_position_steps"],
                "state": sample["state"],
            }
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def clear_plots(self):
        self.plot.clear()

    def toggle_recording(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.record_button.configure(text="Start CSV")
            self.status_var.set("CSV recording stopped")
            return
        path = filedialog.asksaveasfilename(
            title="Save telemetry",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=time.strftime("balance_%Y%m%d_%H%M%S.csv"),
        )
        if not path:
            return
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.host_start_time = time.monotonic()
        self.record_button.configure(text="Stop CSV")
        self.status_var.set(f"Recording: {path}")

    def on_close(self):
        if self.csv_file is not None:
            self.csv_file.close()
        self.worker.disconnect()
        self.destroy()


if __name__ == "__main__":
    TuningApp().mainloop()
