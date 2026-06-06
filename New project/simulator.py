import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import asdict, dataclass, field, fields


DT = 0.1
G = 9.80665


@dataclass
class Anchor:
    id: str
    x: float
    y: float
    z: float


def default_anchors():
    return [
        Anchor("A1", 0.5, 0.5, 2.0),
        Anchor("A2", 9.5, 0.5, 2.0),
        Anchor("A3", 9.5, 9.5, 2.0),
        Anchor("A4", 0.5, 9.5, 2.0),
        Anchor("A5", 5.0, 5.0, 2.5),
    ]


@dataclass
class SimConfig:
    dt: float = DT
    map_width: float = 10.0
    map_height: float = 10.0
    ppr: int = 36
    gear_ratio: float = 15.0
    wheel_radius: float = 0.0762
    lx: float = 0.400
    ly: float = 0.440
    uwb_default_std: float = 0.15
    uwb_min_std: float = 0.05
    uwb_max_std: float = 1.00
    tag_offset_x: float = 0.0
    tag_offset_y: float = 0.0
    tag_offset_z: float = 0.5
    gyro_noise_std: float = 0.01
    acc_noise_std_g: float = 0.015
    wheel_tick_noise_std: float = 0.35
    manual_speed: float = 0.7
    manual_turn_rate: float = 0.9
    anchors: list[Anchor] = field(default_factory=default_anchors)

    @property
    def wheel_ppr(self) -> float:
        return self.ppr * self.gear_ratio

    @property
    def meter_per_tick(self) -> float:
        return 2.0 * math.pi * self.wheel_radius / self.wheel_ppr

    @property
    def rotation_radius_sum(self) -> float:
        return self.lx + self.ly


DEFAULT_SETTINGS = {
    "rate_hz": 10.0,
    "start_x": 1.0,
    "start_y": 1.0,
    "serial_port": None,
    "baudrate": 115200,
    "map_width": 10.0,
    "map_height": 10.0,
    "manual_speed": 0.7,
    "manual_turn_rate": 0.9,
    "uwb_default_std": 0.15,
    "uwb_min_std": 0.05,
    "uwb_max_std": 1.0,
    "tag_offset_x": 0.0,
    "tag_offset_y": 0.0,
    "tag_offset_z": 0.5,
    "gyro_noise_std": 0.01,
    "acc_noise_std_g": 0.015,
    "wheel_tick_noise_std": 0.35,
    "ppr": 36,
    "gear_ratio": 15.0,
    "wheel_radius": 0.0762,
    "lx": 0.400,
    "ly": 0.440,
    "anchors": [asdict(anchor) for anchor in default_anchors()],
}


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def make_motion(t):
    """Return body-frame vx, vy, omega for a smooth repeatable test path."""
    vx = 0.45 + 0.18 * math.sin(0.35 * t)
    vy = 0.22 * math.sin(0.55 * t)
    omega = 0.35 * math.sin(0.25 * t)
    return vx, vy, omega


def world_to_body(wx, wy, theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    vx = cos_t * wx + sin_t * wy
    vy = -sin_t * wx + cos_t * wy
    return vx, vy


def body_to_world(vx, vy, theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    wx = cos_t * vx - sin_t * vy
    wy = sin_t * vx + cos_t * vy
    return wx, wy


def tag_world_position(x, y, theta, config):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    tx = x + cos_t * config.tag_offset_x - sin_t * config.tag_offset_y
    ty = y + sin_t * config.tag_offset_x + cos_t * config.tag_offset_y
    tz = config.tag_offset_z
    return tx, ty, tz


def mecanum_ticks(vx, vy, omega, config):
    # X-layout mecanum inverse kinematics, wheel order: FL, FR, RL, RR.
    l = config.rotation_radius_sum
    wheel_distances = [
        (vx - vy - l * omega) * config.dt,
        (vx + vy + l * omega) * config.dt,
        (vx + vy - l * omega) * config.dt,
        (vx - vy + l * omega) * config.dt,
    ]
    ticks = []
    for distance in wheel_distances:
        noisy = distance / config.meter_per_tick
        noisy += random.gauss(0.0, config.wheel_tick_noise_std)
        ticks.append(int(round(noisy)))
    return ticks


def imu_reading(vx, vy, omega, previous_vx, previous_vy, config):
    ax_body = (vx - previous_vx) / config.dt
    ay_body = (vy - previous_vy) / config.dt

    return {
        "acc_x": ax_body / G + random.gauss(0.0, config.acc_noise_std_g),
        "acc_y": ay_body / G + random.gauss(0.0, config.acc_noise_std_g),
        "acc_z": 1.0 + random.gauss(0.0, config.acc_noise_std_g),
        "gyro_x": random.gauss(0.0, config.gyro_noise_std),
        "gyro_y": random.gauss(0.0, config.gyro_noise_std),
        "gyro_z": omega + random.gauss(0.0, config.gyro_noise_std),
    }


def uwb_readings(x, y, theta, config):
    tx, ty, tz = tag_world_position(x, y, theta, config)
    sample_count = min(len(config.anchors), random.randint(2, 4))
    selected = random.sample(config.anchors, sample_count)
    readings = []

    for anchor in selected:
        dx = tx - anchor.x
        dy = ty - anchor.y
        dz = tz - anchor.z
        true_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        std = clamp(
            random.gauss(config.uwb_default_std, config.uwb_default_std * 0.15),
            config.uwb_min_std,
            config.uwb_max_std,
        )
        measured_distance = max(0.0, true_distance + random.gauss(0.0, std))
        readings.append(
            {
                "id": anchor.id,
                "x": anchor.x,
                "y": anchor.y,
                "z": anchor.z,
                "distance": measured_distance,
                "std": std,
            }
        )

    return readings


def build_packet(seq, x, y, theta, imu, ticks, uwb):
    return {
        "seq": seq,
        "state": {
            "x": x,
            "y": y,
            "theta": theta,
            "theta_deg": math.degrees(theta),
        },
        "imu": imu,
        "hall_ticks": {
            "front_left": ticks[0],
            "front_right": ticks[1],
            "rear_left": ticks[2],
            "rear_right": ticks[3],
        },
        "uwb": uwb,
    }


def make_writer(port, baudrate):
    if not port:
        return None

    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is not installed. Install it with: pip install pyserial"
        ) from exc

    return serial.Serial(port=port, baudrate=baudrate, timeout=1)


def app_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def default_config_path():
    return os.path.join(app_dir(), "sim_config.json")


def write_default_settings(path):
    with open(path, "w", encoding="utf-8") as file:
        json.dump(DEFAULT_SETTINGS, file, indent=2)
        file.write("\n")


def load_settings(path):
    if not os.path.exists(path):
        write_default_settings(path)
        return dict(DEFAULT_SETTINGS)

    try:
        with open(path, "r", encoding="utf-8") as file:
            loaded = json.load(file)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON config file: {path}\n{exc}") from exc

    settings = dict(DEFAULT_SETTINGS)
    settings.update(loaded)
    return settings


def anchors_from_settings(settings):
    anchors = []
    for item in settings.get("anchors", []):
        anchors.append(
            Anchor(
                str(item["id"]),
                float(item["x"]),
                float(item["y"]),
                float(item["z"]),
            )
        )

    if len(anchors) < 2:
        raise SystemExit("Config must contain at least 2 UWB anchors.")
    return anchors


def config_from_settings(settings):
    config_keys = {item.name for item in fields(SimConfig)}
    kwargs = {}

    for key in config_keys:
        if key == "anchors":
            kwargs[key] = anchors_from_settings(settings)
        elif key == "dt":
            rate_hz = float(settings.get("rate_hz", 10.0))
            if rate_hz <= 0:
                raise SystemExit("rate_hz must be greater than 0.")
            kwargs[key] = 1.0 / rate_hz
        elif key in settings:
            kwargs[key] = settings[key]

    return SimConfig(**kwargs)


def apply_cli_overrides(settings, args):
    override_map = {
        "port": "serial_port",
        "baudrate": "baudrate",
        "rate": "rate_hz",
        "uwb_std": "uwb_default_std",
        "tag_x": "tag_offset_x",
        "tag_y": "tag_offset_y",
        "tag_z": "tag_offset_z",
        "start_x": "start_x",
        "start_y": "start_y",
        "speed": "manual_speed",
        "turn_rate": "manual_turn_rate",
    }

    for arg_name, setting_name in override_map.items():
        value = getattr(args, arg_name)
        if value is not None:
            settings[setting_name] = value

    return settings


def parse_args():
    parser = argparse.ArgumentParser(description="Mecanum robot sensor simulator")
    parser.add_argument("--ui", action="store_true", help="Open keyboard control UI")
    parser.add_argument("--auto", action="store_true", help="Run automatic path output mode")
    parser.add_argument("--config", help="JSON config path. Default: sim_config.json next to exe/script")
    parser.add_argument("--port", help="Serial port name, for example COM3")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--rate", type=float, help="Output rate in Hz")
    parser.add_argument("--seed", type=int, help="Random seed for repeatable output")
    parser.add_argument("--uwb-std", type=float, help="Default UWB std in m")
    parser.add_argument("--tag-x", type=float, help="UWB tag offset x in m")
    parser.add_argument("--tag-y", type=float, help="UWB tag offset y in m")
    parser.add_argument("--tag-z", type=float, help="UWB tag height/offset z in m")
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--speed", type=float, help="Manual translation speed in m/s")
    parser.add_argument("--turn-rate", type=float, help="Manual turn rate in rad/s")
    parser.add_argument("--max-lines", type=int, help="Stop after N packets")
    return parser.parse_args()


class Simulator:
    def __init__(self, config, writer, start_x, start_y):
        self.config = config
        self.writer = writer
        self.x = start_x
        self.y = start_y
        self.theta = 0.0
        self.previous_vx = 0.0
        self.previous_vy = 0.0
        self.seq = 0
        self.last_packet = None

    def step_body(self, vx, vy, omega):
        theta_mid = self.theta + omega * self.config.dt * 0.5
        wx, wy = body_to_world(vx, vy, theta_mid)

        self.x = clamp(self.x + wx * self.config.dt, 0.0, self.config.map_width)
        self.y = clamp(self.y + wy * self.config.dt, 0.0, self.config.map_height)
        self.theta = wrap_pi(self.theta + omega * self.config.dt)

        imu = imu_reading(
            vx, vy, omega, self.previous_vx, self.previous_vy, self.config
        )
        ticks = mecanum_ticks(vx, vy, omega, self.config)
        uwb = uwb_readings(self.x, self.y, self.theta, self.config)
        packet = build_packet(self.seq, self.x, self.y, self.theta, imu, ticks, uwb)
        self.output(packet)

        self.previous_vx = vx
        self.previous_vy = vy
        self.seq += 1
        self.last_packet = packet
        return packet

    def step_world(self, wx, wy, omega):
        vx, vy = world_to_body(wx, wy, self.theta)
        return self.step_body(vx, vy, omega)

    def output(self, packet):
        line = json.dumps(packet, separators=(",", ":")) + "\n"
        if self.writer:
            self.writer.write(line.encode("utf-8"))
        elif sys.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()


def run_auto(sim, max_lines):
    try:
        while max_lines is None or sim.seq < max_lines:
            t = sim.seq * sim.config.dt
            vx, vy, omega = make_motion(t)
            sim.step_body(vx, vy, omega)
            time.sleep(sim.config.dt)
    except KeyboardInterrupt:
        pass
    finally:
        if sim.writer:
            sim.writer.close()


def run_ui(sim, config_path):
    if getattr(sys, "frozen", False):
        base_dir = getattr(sys, "_MEIPASS", os.path.dirname(sys.executable))
        os.environ.setdefault("TCL_LIBRARY", os.path.join(base_dir, "tcl", "tcl8.6"))
        os.environ.setdefault("TK_LIBRARY", os.path.join(base_dir, "tcl", "tk8.6"))
        if base_dir not in sys.path:
            sys.path.insert(0, base_dir)

    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:
        raise SystemExit("tkinter is not available in this Python installation.") from exc

    root = tk.Tk()
    root.title("Mecanum Sensor Simulator")
    root.geometry("900x680")
    root.minsize(760, 560)

    keys = set()
    margin = 36
    canvas_size = 560
    scale = (canvas_size - margin * 2) / sim.config.map_width

    main = ttk.Frame(root, padding=10)
    main.pack(fill="both", expand=True)

    canvas = tk.Canvas(main, width=canvas_size, height=canvas_size, bg="#f8fafc")
    canvas.grid(row=0, column=0, rowspan=2, sticky="nsew")

    side = ttk.Frame(main, padding=(12, 0, 0, 0))
    side.grid(row=0, column=1, sticky="nsew")
    main.columnconfigure(0, weight=1)
    main.rowconfigure(0, weight=1)

    state_var = tk.StringVar()
    imu_var = tk.StringVar()
    tick_var = tk.StringVar()
    uwb_var = tk.StringVar()
    if sim.writer:
        output_text = "Output: serial"
    elif sys.stdout:
        output_text = "Output: stdout"
    else:
        output_text = "Output: UI only"

    output_var = tk.StringVar(value=f"{output_text}\nConfig: {config_path}")

    ttk.Label(side, text="Controls", font=("Segoe UI", 13, "bold")).pack(anchor="w")
    ttk.Label(side, text="W/S: forward/back\nA/D: strafe left/right\nZ/X: CCW/CW").pack(anchor="w", pady=(4, 14))
    ttk.Label(side, textvariable=output_var).pack(anchor="w", pady=(0, 14))
    ttk.Label(side, text="State", font=("Segoe UI", 12, "bold")).pack(anchor="w")
    ttk.Label(side, textvariable=state_var, justify="left").pack(anchor="w", pady=(4, 12))
    ttk.Label(side, text="IMU", font=("Segoe UI", 12, "bold")).pack(anchor="w")
    ttk.Label(side, textvariable=imu_var, justify="left").pack(anchor="w", pady=(4, 12))
    ttk.Label(side, text="Hall Ticks", font=("Segoe UI", 12, "bold")).pack(anchor="w")
    ttk.Label(side, textvariable=tick_var, justify="left").pack(anchor="w", pady=(4, 12))
    ttk.Label(side, text="UWB", font=("Segoe UI", 12, "bold")).pack(anchor="w")
    ttk.Label(side, textvariable=uwb_var, justify="left").pack(anchor="w", pady=(4, 12))

    def to_canvas(x, y):
        return margin + x * scale, canvas_size - margin - y * scale

    def draw_grid():
        canvas.delete("all")
        x0, y0 = to_canvas(0.0, 0.0)
        x1, y1 = to_canvas(sim.config.map_width, sim.config.map_height)
        canvas.create_rectangle(x0, y1, x1, y0, outline="#334155", width=2)

        for i in range(11):
            gx, _ = to_canvas(i, 0)
            _, gy = to_canvas(0, i)
            canvas.create_line(gx, y0, gx, y1, fill="#e2e8f0")
            canvas.create_line(x0, gy, x1, gy, fill="#e2e8f0")
            canvas.create_text(gx, y0 + 16, text=str(i), fill="#64748b", font=("Segoe UI", 8))
            canvas.create_text(x0 - 16, gy, text=str(i), fill="#64748b", font=("Segoe UI", 8))

        for anchor in sim.config.anchors:
            ax, ay = to_canvas(anchor.x, anchor.y)
            canvas.create_oval(ax - 7, ay - 7, ax + 7, ay + 7, fill="#2563eb", outline="")
            canvas.create_text(ax, ay - 16, text=anchor.id, fill="#1e3a8a", font=("Segoe UI", 9, "bold"))

    def draw_robot(packet):
        draw_grid()
        rx, ry = to_canvas(sim.x, sim.y)
        radius = 15
        canvas.create_oval(rx - radius, ry - radius, rx + radius, ry + radius, fill="#16a34a", outline="#14532d", width=2)

        hx = rx + math.cos(sim.theta) * 30
        hy = ry - math.sin(sim.theta) * 30
        canvas.create_line(rx, ry, hx, hy, fill="#052e16", width=3, arrow=tk.LAST)

        tx, ty, _ = tag_world_position(sim.x, sim.y, sim.theta, sim.config)
        tcx, tcy = to_canvas(tx, ty)
        canvas.create_oval(tcx - 4, tcy - 4, tcx + 4, tcy + 4, fill="#f97316", outline="")

        for reading in packet["uwb"]:
            ax, ay = to_canvas(reading["x"], reading["y"])
            canvas.create_line(tcx, tcy, ax, ay, fill="#f97316", dash=(4, 4))

    def update_labels(packet):
        state = packet["state"]
        imu = packet["imu"]
        ticks = packet["hall_ticks"]
        uwb = packet["uwb"]
        state_var.set(
            f"x: {state['x']:.3f} m\n"
            f"y: {state['y']:.3f} m\n"
            f"theta: {state['theta']:.3f} rad\n"
            f"theta_deg: {state['theta_deg']:.1f} deg"
        )
        imu_var.set(
            f"acc: {imu['acc_x']:.3f}, {imu['acc_y']:.3f}, {imu['acc_z']:.3f} g\n"
            f"gyro: {imu['gyro_x']:.3f}, {imu['gyro_y']:.3f}, {imu['gyro_z']:.3f} rad/s"
        )
        tick_var.set(
            f"FL: {ticks['front_left']}\nFR: {ticks['front_right']}\n"
            f"RL: {ticks['rear_left']}\nRR: {ticks['rear_right']}"
        )
        uwb_lines = [
            f"{u['id']} ({u['x']:.1f},{u['y']:.1f},{u['z']:.1f}) d={u['distance']:.2f} std={u['std']:.2f}"
            for u in uwb
        ]
        uwb_var.set("\n".join(uwb_lines))

    def command_velocity():
        speed = sim.config.manual_speed
        turn = sim.config.manual_turn_rate
        vx = 0.0
        vy = 0.0
        omega = 0.0

        if "w" in keys:
            vx += speed
        if "s" in keys:
            vx -= speed
        if "a" in keys:
            vy += speed
        if "d" in keys:
            vy -= speed
        if "z" in keys:
            omega += turn
        if "x" in keys:
            omega -= turn

        if vx and vy:
            inv = 1.0 / math.sqrt(2.0)
            vx *= inv
            vy *= inv

        return vx, vy, omega

    def tick():
        vx, vy, omega = command_velocity()
        packet = sim.step_body(vx, vy, omega)
        draw_robot(packet)
        update_labels(packet)
        root.after(int(sim.config.dt * 1000), tick)

    def on_key_press(event):
        key = event.keysym.lower()
        if key in {"w", "a", "s", "d", "z", "x"}:
            keys.add(key)

    def on_key_release(event):
        key = event.keysym.lower()
        keys.discard(key)

    def on_close():
        if sim.writer:
            sim.writer.close()
        root.destroy()

    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.focus_set()

    draw_grid()
    tick()
    root.mainloop()


def main():
    args = parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    config_path = os.path.abspath(args.config or default_config_path())
    settings = apply_cli_overrides(load_settings(config_path), args)
    config = config_from_settings(settings)
    writer = make_writer(settings.get("serial_port"), int(settings["baudrate"]))
    sim = Simulator(config, writer, float(settings["start_x"]), float(settings["start_y"]))

    default_ui = getattr(sys, "frozen", False) and not args.auto

    if args.ui or default_ui:
        run_ui(sim, config_path)
    else:
        run_auto(sim, args.max_lines)


if __name__ == "__main__":
    main()
