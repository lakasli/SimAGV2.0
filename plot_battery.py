import argparse
import json
import time
from collections import deque

import paho.mqtt.client as mqtt


def parse_args():
    parser = argparse.ArgumentParser(description="Subscribe to AGV state and plot batteryCharge in real time.")
    parser.add_argument("--host", default="127.0.0.1", help="MQTT broker host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: 1883)")
    parser.add_argument(
        "--topic",
        default="uagv/v2/SEER/AMB-01/state",
        help="MQTT topic to subscribe (default: uagv/v2/SEER/AMB-01/state)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=600,
        help="Plot window in seconds (default: 600 seconds)",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=10000,
        help="Max number of points to retain (default: 10000)",
    )
    return parser.parse_args()


class BatteryPlotter:
    def __init__(self, window_seconds: int = 600, max_points: int = 10000):
        try:
            import matplotlib.pyplot as plt  # noqa: F401
            import matplotlib.animation as animation  # noqa: F401
        except Exception as e:
            raise RuntimeError(
                "matplotlib is required. Please install it: python -m pip install matplotlib"
            ) from e
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation

        self.plt = plt
        self.animation = animation
        self.start_ts = time.time()
        self.window = float(window_seconds)
        self.times = deque(maxlen=max_points)  # seconds since start
        self.values = deque(maxlen=max_points)  # batteryCharge

        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        (self.line,) = self.ax.plot([], [], color="#4a90e2", lw=2, label="batteryCharge %")
        self.ax.set_title("AGV Battery Charge (%)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Battery (%)")
        self.ax.set_ylim(0, 100)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right")

        # last value annotation
        self.text_last = self.ax.text(0.98, 0.02, "", transform=self.ax.transAxes, ha="right", va="bottom")

        self.ani = self.animation.FuncAnimation(
            self.fig, self._update_plot, interval=200, blit=False
        )

    def add_point(self, battery_charge: float):
        now = time.time()
        t = now - self.start_ts
        # clamp to [0, 100]
        try:
            v = float(battery_charge)
        except Exception:
            return
        if not (v == v):  # NaN
            return
        v = max(0.0, min(100.0, v))
        self.times.append(t)
        self.values.append(v)

    def _update_plot(self, _frame):
        if not self.times:
            return self.line
        # limit to window
        tmax = self.times[-1]
        tmin = max(0.0, tmax - self.window)
        # find index of first t >= tmin
        idx = 0
        for i, tt in enumerate(self.times):
            if tt >= tmin:
                idx = i
                break
        xs = list(self.times)[idx:]
        ys = list(self.values)[idx:]
        self.line.set_data(xs, ys)
        self.ax.set_xlim(tmin, max(tmin + 10.0, tmax))
        # update last value text
        if ys:
            self.text_last.set_text(f"{ys[-1]:.1f}%")
        return self.line

    def show(self):
        self.plt.show()


def main():
    args = parse_args()

    plotter = BatteryPlotter(window_seconds=args.window, max_points=args.max_points)

    def on_connect(client: mqtt.Client, _userdata, _flags, _rc, _properties=None):
        print(f"Connected to MQTT {args.host}:{args.port}, subscribing {args.topic}")
        client.subscribe(args.topic, qos=1)

    def on_message(_client: mqtt.Client, _userdata, msg: mqtt.MQTTMessage):
        try:
            payload = msg.payload.decode("utf-8", errors="ignore")
            data = json.loads(payload)
            bs = data.get("batteryState") or {}
            v = bs.get("batteryCharge")
            if v is None:
                return
            plotter.add_point(float(v))
        except Exception as e:
            print(f"Parse message failed: {e}")

    client = mqtt.Client(protocol=mqtt.MQTTv5)
    client.on_connect = on_connect
    client.on_message = on_message
    # connect and start loop in background
    client.connect(args.host, args.port, keepalive=60)
    client.loop_start()

    try:
        plotter.show()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()