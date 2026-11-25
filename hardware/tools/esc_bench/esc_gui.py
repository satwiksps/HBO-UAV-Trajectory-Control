"""Live throttle panel for the A2212 ESC bench setup."""
import argparse
import time
import tkinter as tk
from tkinter import ttk

from esc_link import send, serial
from serial.tools import list_ports


BACKGROUND = "#101923"
PANEL = "#1A2734"
EDGE = "#334759"
TEXT = "#F3F7FA"
MUTED = "#ADBFCE"
MINT = "#62DFB0"
AMBER = "#FFD28A"
RED = "#FF8289"
FONT = "Segoe UI"
PIN = "D6"
IDLE_US = 950
START_US = 1070  # Just above the 1062 µs onset reported on this bench.
LIMIT_US = 1300
IDLE_STATES = {"IDLE", "TIMEOUT", "LIMIT"}


class ThrottlePanel:
    def __init__(self, root, preferred_port="COM5"):
        self.root = root
        self.port = None
        self.phase = "offline"
        self.receive_buffer = bytearray()
        self.last_status = 0.0
        self.next_query = 0.0
        self.next_command = 0.0
        self.ready_at = 0.0
        self.deadline = 0.0
        self.pulse = None
        self.preferred_port = preferred_port

        self.port_name = tk.StringVar(value=preferred_port)
        self.percent = tk.IntVar(value=0)
        self.percent_text = tk.StringVar(value="0%")
        self.output_text = tk.StringVar(value="—")
        self.status_text = tk.StringVar(value="Disconnected")
        self.detail_text = tk.StringVar(value="Select the Uno’s port and connect.")

        root.title("Motor Bench")
        root.configure(bg=BACKGROUND)
        root.geometry("665x515")
        root.minsize(630, 490)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<KeyPress-space>", self.stop_key)

        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("Motor.TCombobox", bordercolor=EDGE, arrowcolor=TEXT,
                        fieldbackground=PANEL, background=PANEL, foreground=TEXT,
                        selectbackground=PANEL, selectforeground=TEXT, padding=6)
        style.map("Motor.TCombobox",
                  fieldbackground=[("readonly", PANEL), ("disabled", BACKGROUND)],
                  foreground=[("disabled", MUTED), ("readonly", TEXT)])
        self.draw()
        self.refresh_ports()
        self.render()
        self.root.after(40, self.poll)

    def label(self, parent, *, text="", variable=None, size=11, color=TEXT,
              weight="normal", bg=BACKGROUND, **kwargs):
        return tk.Label(parent, text=text, textvariable=variable, anchor="w",
                        bg=bg, fg=color, font=(FONT, size, weight), **kwargs)

    def button(self, parent, text, command, bg, fg=TEXT, **kwargs):
        button = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                           activebackground=fg, activeforeground=bg,
                           disabledforeground="#8C9EAC", font=(FONT, 10, "bold"),
                           padx=16, pady=8, bd=0, highlightthickness=1,
                           highlightbackground=bg, highlightcolor=TEXT, **kwargs)
        button.bind("<KeyPress-space>", self.stop_key)
        return button

    def draw(self):
        page = tk.Frame(self.root, bg=BACKGROUND, padx=26, pady=20)
        page.pack(fill="both", expand=True)
        title_row = tk.Frame(page, bg=BACKGROUND)
        title_row.pack(fill="x")
        self.label(title_row, text="Motor Bench", size=25, weight="bold").pack(side="left")
        self.status_label = self.label(title_row, variable=self.status_text,
                                       color=MUTED, weight="bold")
        self.status_label.pack(side="right", padx=(12, 0))
        self.label(page, text="A2212 brushless motor  /  SimonK ESC  /  Uno D6",
                   color=MUTED).pack(fill="x", pady=(2, 14))

        connection = tk.Frame(page, bg=BACKGROUND)
        connection.pack(fill="x", pady=(0, 14))
        self.label(connection, text="Port", color=MUTED).pack(side="left", padx=(0, 8))
        self.port_box = ttk.Combobox(connection, textvariable=self.port_name,
                                     state="readonly", width=10, style="Motor.TCombobox",
                                     font=(FONT, 11), postcommand=self.refresh_ports)
        self.port_box.pack(side="left", padx=(0, 9))
        self.port_box.bind("<KeyPress-space>", self.stop_key)
        self.connect_button = self.button(connection, "Connect", self.toggle_link, EDGE)
        self.connect_button.pack(side="left")
        self.label(connection, text="Propeller off · Motor mounted", color=AMBER,
                   size=10).pack(side="right")

        panel = tk.Frame(page, bg=PANEL, padx=17, pady=14,
                         highlightbackground=EDGE, highlightthickness=1)
        panel.pack(fill="both", expand=True)
        values = tk.Frame(panel, bg=PANEL)
        values.pack(fill="x")
        request = tk.Frame(values, bg=PANEL)
        request.pack(side="left")
        output = tk.Frame(values, bg=PANEL)
        output.pack(side="right")
        self.label(request, text="Throttle command", color=MUTED, bg=PANEL).pack(anchor="w")
        self.label(request, variable=self.percent_text, size=32, weight="bold",
                   color=MINT, bg=PANEL).pack(anchor="w")
        self.label(output, text="Uno output command", color=MUTED, bg=PANEL).pack(anchor="e")
        self.label(output, variable=self.output_text, size=24, weight="bold",
                   bg=PANEL).pack(anchor="e", pady=(4, 0))

        self.slider = tk.Scale(panel, from_=0, to=100, orient="horizontal",
                               showvalue=False, variable=self.percent, resolution=1,
                               takefocus=True, highlightthickness=1, highlightbackground=PANEL,
                               highlightcolor=MINT, bg=PANEL, fg=TEXT, troughcolor="#0D1620",
                               activebackground=MINT, sliderlength=29, width=16, bd=0,
                               command=self.show_requested)
        self.slider.pack(fill="x", pady=(8, 0))
        self.slider.bind("<KeyPress-space>", self.stop_key)
        ends = tk.Frame(panel, bg=PANEL)
        ends.pack(fill="x")
        self.label(ends, text=f"0% · Idle · {IDLE_US} µs", color=MUTED, bg=PANEL,
                   size=10).pack(side="left")
        self.label(ends, text=f"100% · Bench limit · {LIMIT_US} µs", color=MUTED, bg=PANEL,
                   size=10).pack(side="right")

        actions = tk.Frame(panel, bg=PANEL)
        actions.pack(fill="x", pady=(15, 0))
        self.enable_button = self.button(actions, "Enable throttle", self.enable, MINT, "#10251D")
        self.enable_button.pack(side="left")
        self.stop_button = self.button(actions, "STOP  ·  Space", self.stop, RED, "#2E1417")
        self.stop_button.pack(side="right")
        self.label(page, variable=self.detail_text, color=MUTED, size=10,
                   wraplength=565, justify="left").pack(fill="x", pady=(9, 0))
        self.label(page, text=f"1% sends {START_US} µs. RPM is unmeasured.",
                   color=MUTED, size=9).pack(fill="x", pady=(4, 0))

    def refresh_ports(self):
        seen = [item.device for item in list_ports.comports()]
        self.port_box.configure(values=seen)
        if self.port_name.get() not in seen:
            self.port_name.set(self.preferred_port if self.preferred_port in seen
                               else (seen[0] if seen else ""))

    def set_phase(self, phase, detail):
        self.phase = phase
        self.detail_text.set(detail)
        self.render()

    def render(self):
        title, color = {
            "offline": ("Disconnected", MUTED),
            "connecting": ("Connecting…", AMBER),
            "idle": ("Connected · Idle", MINT),
            "enabling": ("Enabling…", AMBER),
            "live": ("Live control", MINT),
            "stopping": ("Stopping…", AMBER),
        }[self.phase]
        self.status_text.set(title)
        self.status_label.configure(fg=color)
        self.connect_button.configure(text="Disconnect" if self.port else "Connect")
        self.port_box.configure(state="disabled" if self.port else "readonly")
        self.enable_button.configure(state="normal" if self.phase == "idle" else "disabled")
        self.stop_button.configure(state="normal" if self.port else "disabled")
        self.slider.configure(state="normal" if self.phase == "live" else "disabled")
        self.output_text.set(f"{self.pulse} µs" if self.pulse is not None else "—")

    def show_requested(self, _value=None):
        self.percent_text.set(f"{self.percent.get()}%")

    def zero_slider(self):
        self.percent.set(0)
        self.show_requested()

    def toggle_link(self):
        if self.port:
            self.disconnect("Disconnected. Throttle returns to idle.")
        else:
            self.connect()

    def connect(self):
        if self.port:
            return
        self.refresh_ports()
        name = self.port_name.get()
        if not name:
            self.set_phase("offline", "No serial port detected. Plug in the Uno and try Connect.")
            return
        port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.15)
        port.port = name
        port.dtr = False
        port.rts = False
        try:
            port.open()
            port.reset_input_buffer()
        except OSError as error:
            port.close()
            self.set_phase("offline", f"Could not open {name}. {error}")
            return
        self.port = port
        self.receive_buffer.clear()
        now = time.monotonic()
        self.ready_at = now + 5.8
        self.deadline = now + 8.0
        self.next_query = now + 1.0
        self.last_status = now
        self.set_phase("connecting", f"Waiting for the D6 controller on {name}.")

    def disconnect(self, detail, *, stop=True):
        port, self.port = self.port, None
        if port:
            try:
                if stop:
                    send(port, "STOP")
            except OSError:
                pass
            finally:
                port.close()
        self.pulse = None
        self.zero_slider()
        self.set_phase("offline", detail)

    def enable(self):
        if self.phase != "idle" or not self.port:
            return
        self.zero_slider()
        self.deadline = time.monotonic() + 0.7
        self.set_phase("enabling", "Waiting for the Uno to enable live control at idle.")
        try:
            send(self.port, "LIVE")
        except OSError:
            self.disconnect("USB connection lost. The Uno’s timeout returns it to idle.")

    def stop_key(self, _event):
        self.stop()
        return "break"

    def stop(self):
        self.zero_slider()
        if not self.port:
            return
        self.deadline = time.monotonic() + 0.8
        self.set_phase("stopping", "Sending STOP. Waiting for idle confirmation.")
        try:
            send(self.port, "STOP")
        except OSError:
            self.disconnect("USB connection lost. The Uno’s timeout returns it to idle.")

    def handle_status(self, line, now):
        fields = line.split(",")
        if (fields[0], len(fields)) in (("ARM2", 19), ("ARM3", 23)):
            self.disconnect(
                "The Uno is running the Propeller Arm PID firmware. "
                "Use Open Propeller Arm or Validate HBO Candidate for this rig.",
                stop=False,
            )
            return
        if len(fields) != 6 or fields[0] != "ESC_LIVE_V1":
            return
        if fields[1:4] != [PIN, str(IDLE_US), str(LIMIT_US)]:
            self.disconnect("The Uno must have the D6 bench firmware with the 950–1300 µs range.")
            return
        try:
            pulse = int(fields[4])
        except ValueError:
            return
        mode = fields[5]
        if not IDLE_US <= pulse <= LIMIT_US or mode not in IDLE_STATES | {"LIVE", "RAMP"}:
            return
        self.last_status = now
        self.pulse = pulse
        self.output_text.set(f"{pulse} µs")
        if mode == "LIVE":
            if self.phase == "enabling":
                self.next_command = now
                self.set_phase("live", "Move the slider to set throttle. Space returns to idle.")
            elif self.phase not in {"live", "stopping"}:
                self.stop()
        elif mode == "RAMP":
            if self.phase != "stopping":
                self.stop()
        elif self.phase == "connecting" and now < self.ready_at:
            return
        elif self.phase == "enabling" and mode == "IDLE":
            return
        elif self.phase != "idle":
            self.zero_slider()
            detail = {
                "IDLE": "Uno at idle. Enable throttle when the ESC is ready.",
                "TIMEOUT": "Uno returned to idle after losing throttle updates.",
                "LIMIT": "Three-minute run limit reached. Enable again to continue.",
            }[mode]
            self.set_phase("idle", detail)

    def read_status(self, now):
        self.receive_buffer.extend(self.port.read(min(self.port.in_waiting, 1024)))
        while b"\n" in self.receive_buffer:
            line, _, self.receive_buffer = self.receive_buffer.partition(b"\n")
            self.handle_status(line.decode("ascii", errors="replace").strip(), now)
            if not self.port:
                return
        if len(self.receive_buffer) > 1024:
            self.receive_buffer.clear()

    def poll(self):
        if self.port:
            now = time.monotonic()
            try:
                self.read_status(now)
                if self.port:
                    self.service_link(now)
            except OSError:
                self.disconnect("USB connection lost. The Uno’s timeout returns it to idle.")
        self.root.after(40, self.poll)

    def service_link(self, now):
        if self.phase == "connecting" and now >= self.deadline:
            self.disconnect("The Uno did not answer with the live throttle firmware.")
            return
        if self.phase != "connecting" and now - self.last_status >= 0.8:
            self.disconnect("Uno status timed out. The Uno’s timeout returns it to idle.")
            return
        if self.phase in {"enabling", "stopping"} and now >= self.deadline:
            self.disconnect("The Uno did not confirm the command. Reconnect to verify idle.")
            return
        if self.phase == "live" and now >= self.next_command:
            percent = self.percent.get()
            requested = IDLE_US if percent == 0 else START_US + round(
                (LIMIT_US - START_US) * (percent - 1) / 99
            )
            send(self.port, f"SET {requested}")
            self.next_command = now + 0.1
        if now >= self.next_query:
            send(self.port, "STATE")
            self.next_query = now + 0.2

    def close(self):
        self.disconnect("")
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--connect", action="store_true", help="Connect on launch and stay at idle.")
    args = parser.parse_args()
    root = tk.Tk()
    panel = ThrottlePanel(root, args.port)
    if args.connect:
        root.after(250, panel.connect)
    root.mainloop()


if __name__ == "__main__":
    main()
