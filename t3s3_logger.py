#!/usr/bin/env python3
"""
t3s3_logger.py - GUI serial logger for the Heltec V4 SAW test.

Connects to the LilyGO T3-S3 reference receiver (115200 baud), filters the
CSV stream for the selected transmitter (R2 or R8) and records each packet
into a per-device wide-format CSV file, one column block per test distance:

    500m_Packet,500m_RSSI,500m_SNR,1000m_Packet,1000m_RSSI,...,2500m_SNR

One row = one packet number (row 1 = packet 000001 ... row 100 = packet
000100). Only the block for the distance currently being tested is filled;
the other blocks keep their data from earlier sessions.

A session auto-stops when 100 unique packets have been recorded for the
selected distance (the 100-packet test procedure). The serial port stays
open between sessions, so you can switch distance and press Start again.

Requirements:
    Python 3.8+
    pyserial          ->  pip install pyserial
    tkinter           ->  included with standard Python on Windows/macOS
"""

import csv
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing - install it with:  pip install pyserial")

DISTANCES = [500, 1000, 1500, 2000, 2500]
PACKETS = 100                 # packets per run (test procedure)
BAUD = 115200
SETTLE_MS = 2500              # wait after opening the port (receiver may reboot)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def csv_header():
    cols = []
    for d in DISTANCES:
        cols += ["%dm_Packet" % d, "%dm_RSSI" % d, "%dm_SNR" % d]
    return cols


class DataFile:
    """Wide CSV, one file per device (R2.csv / R8.csv), 100 rows."""

    def __init__(self, device, path=None):
        self.device = device
        self.path = path or os.path.join(BASE_DIR, "%s.csv" % device)
        self.grid = [[""] * (len(DISTANCES) * 3) for _ in range(PACKETS)]
        self._load()

    def _load(self):
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, newline="") as f:
                rows = list(csv.reader(f))
        except OSError:
            return
        if not rows:
            return
        data = rows[1:] if rows and rows[0] and rows[0][0].endswith("_Packet") else rows
        for i, row in enumerate(data[:PACKETS]):
            if len(row) == len(self.grid[0]):
                self.grid[i] = row

    def store(self, dist, seq, rssi, snr):
        base = DISTANCES.index(dist) * 3
        row = self.grid[seq - 1]
        row[base] = str(seq)
        row[base + 1] = rssi
        row[base + 2] = snr
        self.save()

    def filled(self, dist):
        base = DISTANCES.index(dist) * 3
        return sum(1 for r in self.grid if r[base] != "")

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(csv_header())
            w.writerows(self.grid)
        os.replace(tmp, self.path)


class App:
    def __init__(self, root):
        self.root = root
        root.title("T3S3 Distance Logger")
        root.minsize(760, 520)

        self.port_var = tk.StringVar()
        self.device_var = tk.StringVar(value="R2")
        self.dist_var = tk.StringVar(value="%d m" % DISTANCES[0])
        self.status_var = tk.StringVar(value="Idle")

        self.ser = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.listening = False
        self.session_seen = set()
        self.session_device = None
        self.session_dist = None
        self.datafile = None

        self._build_ui()
        self.refresh_ports()
        root.after(100, self.poll)

    # ----------------------------------------------------------------- UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Serial port").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     state="readonly", width=34)
        self.port_box.grid(row=1, column=0, sticky="w")
        ttk.Button(top, text="Refresh", command=self.refresh_ports)\
            .grid(row=1, column=1, padx=(6, 18), sticky="w")

        ttk.Label(top, text="Device (writes its CSV)").grid(row=0, column=2,
                                                            sticky="w")
        dev = ttk.Frame(top)
        dev.grid(row=1, column=2, sticky="w")
        for name in ("R2", "R8"):
            ttk.Radiobutton(dev, text=name, value=name,
                            variable=self.device_var).pack(side="left",
                                                           padx=(0, 8))

        ttk.Label(top, text="Test distance").grid(row=0, column=4, sticky="w",
                                                  padx=(18, 0))
        ttk.Combobox(top, textvariable=self.dist_var, state="readonly",
                     width=8,
                     values=["%d m" % d for d in DISTANCES])\
            .grid(row=1, column=4, sticky="w", padx=(18, 0))

        btns = ttk.Frame(top)
        btns.grid(row=1, column=5, padx=(18, 0), sticky="e")
        self.start_btn = ttk.Button(btns, text="Start Listening",
                                    command=self.start)
        self.start_btn.pack(side="left", padx=(0, 6))
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.stop,
                                   state="disabled")
        self.stop_btn.pack(side="left")
        top.columnconfigure(5, weight=1)

        # per-device CSV save locations (portable across PCs)
        csvf = ttk.LabelFrame(self.root, text="CSV save location (per device)",
                              padding=8)
        csvf.pack(fill="x", padx=8, pady=(0, 4))
        self.csv_paths_var = {}
        for i, name in enumerate(("R2", "R8")):
            ttk.Label(csvf, text="%s.csv" % name).grid(
                row=i, column=0, sticky="w", padx=(0, 6))
            var = tk.StringVar(value=os.path.join(BASE_DIR, "%s.csv" % name))
            ttk.Entry(csvf, textvariable=var, state="readonly").grid(
                row=i, column=1, sticky="we", padx=(0, 6), pady=1)
            ttk.Button(csvf, text="Browse...",
                       command=lambda n=name: self.browse_csv(n)).grid(
                row=i, column=2, pady=1)
            self.csv_paths_var[name] = var
        csvf.columnconfigure(1, weight=1)

        # status / progress
        mid = ttk.Frame(self.root, padding=(8, 0))
        mid.pack(fill="x")
        ttk.Label(mid, textvariable=self.status_var).pack(side="left")
        self.progress = ttk.Progressbar(mid, maximum=PACKETS, length=260)
        self.progress.pack(side="right")

        # log area
        logf = ttk.Frame(self.root, padding=8)
        logf.pack(fill="both", expand=True)
        self.log = tk.Text(logf, height=14, wrap="none", state="disabled",
                           font=("Consolas", 9))
        sb = ttk.Scrollbar(logf, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def log_line(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def refresh_ports(self):
        ports = ["%s  (%s)" % (p.device, p.description or "?")
                 for p in list_ports.comports()]
        self.port_box.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def browse_csv(self, device):
        cur = self.csv_paths_var[device].get()
        path = filedialog.asksaveasfilename(
            title="Save %s.csv to" % device,
            initialdir=os.path.dirname(cur) if cur else BASE_DIR,
            initialfile=os.path.basename(cur) if cur else "%s.csv" % device,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if path:
            self.csv_paths_var[device].set(path)
            self.log_line("# %s.csv -> %s" % (device, path))

    # ------------------------------------------------------------- control
    def _set_controls(self):
        state = "disabled" if self.listening else "normal"
        self.start_btn.configure(state=state)
        self.stop_btn.configure(state="normal" if self.listening else "disabled")
        self.port_box.configure(state="disabled" if self.listening else "readonly")

    def start(self):
        if self.listening:
            return
        port = self.port_var.get().split()[0] if self.port_var.get() else ""
        if not port:
            messagebox.showerror("No port", "Select a serial port first.")
            return
        device = self.device_var.get()
        dist = int(self.dist_var.get().split()[0])
        csv_path = self.csv_paths_var[device].get()

        if (self.datafile is None or self.datafile.device != device
                or self.datafile.path != csv_path):
            self.datafile = DataFile(device, csv_path)
        prior = self.datafile.filled(dist)
        if prior:
            self.log_line("# note: %s.csv already has %d packets for %d m "
                          "- they will be overwritten as this run progresses"
                          % (device, prior, dist))

        self.session_device = device
        self.session_dist = dist
        self.session_seen = set()
        self.listening = True
        self._set_controls()
        self.progress["value"] = 0
        self.status_var.set("Listening - %s - %d m - 0/%d"
                            % (device, dist, PACKETS))
        self.log_line("# listening: %s @ %d m -> %s"
                      % (device, dist, self.datafile.path))

        just_opened = False
        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(port, BAUD, timeout=1)
            except serial.SerialException as e:
                self.log_line("# ERROR opening %s: %s" % (port, e))
                self.listening = False
                self._set_controls()
                self.status_var.set("Idle - port error")
                return
            just_opened = True
            self.reader_stop.clear()
            self.reader_thread = threading.Thread(target=self._read_loop,
                                                  daemon=True)
            self.reader_thread.start()
            self.log_line("# opened %s @ %d baud" % (port, BAUD))

        if just_opened:
            self.log_line("# waiting %.1f s for the receiver ..."
                          % (SETTLE_MS / 1000))
            self.root.after(SETTLE_MS, lambda: self._send_setup(dist))
        else:
            self._send_setup(dist)

    def _send_setup(self, dist):
        try:
            self.ser.write(("d%d\n" % dist).encode())
            self.ser.write(b"r\n")
            self.log_line("# sent: d%d (distance) + r (reset stats)" % dist)
        except serial.SerialException as e:
            self.log_line("# ERROR sending commands: %s" % e)

    def stop(self):
        if self.listening:
            self.stop_session("stopped by user (%d/%d packets)"
                              % (len(self.session_seen), PACKETS))

    def stop_session(self, reason):
        self.listening = False
        self._set_controls()
        self.status_var.set("Idle")
        self.log_line("# session ended: %s" % reason)

    # -------------------------------------------------------------- reader
    def _read_loop(self):
        while not self.reader_stop.is_set():
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.2)
                    continue
                raw = self.ser.readline()
                if raw:
                    self.rx_queue.put(raw.decode("utf-8", "replace").rstrip("\r\n"))
            except serial.SerialException as e:
                self.rx_queue.put("## SERIAL ERROR: %s" % e)
                time.sleep(0.5)

    # ------------------------------------------------------------ rx -> csv
    def poll(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def _handle_line(self, line):
        if not line:
            return
        if line.startswith("##"):
            self.log_line(line)
            self.stop_session("serial error")
            if self.ser:
                try:
                    self.ser.close()
                except serial.SerialException:
                    pass
                self.ser = None
            return
        if line.startswith("#"):
            self.log_line(line)
            return
        if not self.listening:
            return                                    # data outside a session

        parts = line.split(",")
        if len(parts) != 5:
            self.log_line("? unparsed: %s" % line)
            return
        if parts[0] != self.session_device:
            return                                    # other transmitter
        try:
            seq = int(parts[2])
            float(parts[3])
            float(parts[4])
        except ValueError:
            self.log_line("? bad values: %s" % line)
            return
        if not 1 <= seq <= PACKETS:
            self.log_line("? packet out of range: %s" % line)
            return

        self.datafile.store(self.session_dist, seq, parts[3], parts[4])
        self.session_seen.add(seq)
        n = len(self.session_seen)
        self.progress["value"] = n
        self.status_var.set("Listening - %s - %d m - %d/%d"
                            % (self.session_device, self.session_dist, n, PACKETS))
        if n >= PACKETS:
            self.stop_session("distance complete: %d/%d packets -> %s"
                              % (n, PACKETS, self.datafile.path))

    # ---------------------------------------------------------------- exit
    def on_close(self):
        if self.listening:
            self.stop_session("window closed")
        self.reader_stop.set()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
