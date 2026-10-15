"""
Pelican Crossing - Modbus TCP Monitor (read-only)
Jetson Orin Nano  --Modbus TCP-->  Siemens S7-1200 (MB_SERVER, port 502)

Turunan dari backend_modbus_only.py: cara baca sama (1x FC03 Reg 0..23, default tiap 1 detik), ditambah
pemeriksaan / alarm di atas data yang dibaca. TIDAK ada penulisan ke PLC.
Satu-satunya dependensi non-stdlib: pymodbus.

Kompatibel program PLC PellicanCrossing v0.7 dan v0.7.1 (peta register sama,
DB_Jetson_Comm.Holding_Data = Array[0..29] of Int).

Peta register yang dibaca (Reg 0..23):
  0 State | 1 Total pelanggaran | 2..7 Timer State 1..6 | 8 Buzzer | 9 Mode standby
  10 Parade lock | 11 Run/Pause | 12 Lompat state | 13 Lampu sorot | 14,15 Pelanggaran sisi 1/2
  16..19 Jadwal lampu sorot | 20 Bitmask output | 21 Detik berjalan | 22 Heartbeat
  23 Flag reset pelanggaran (auto-clear oleh PLC)

Alarm yang dipantau (muncul sekali saat aktif, sekali saat kembali normal):
  CRIT OUTPUT_CONFLICT   Lampu kendaraan hijau/kuning menyala bersamaan dengan hijau pejalan kaki, dll.
  CRIT HEARTBEAT_STALE   Reg 22 tidak berubah > 3 detik (program PLC macet / STOP)
  WARN OUTPUT_MISMATCH   Bitmask output (Reg 20) tidak sesuai dengan state aktif
  WARN STATE_INVALID     State di luar 0..6
  WARN STATE_OVERTIME    Detik berjalan melewati durasi state + 2 detik saat Run
  WARN VIOLATION_SUM     Reg 1 != Reg 14 + Reg 15
  WARN CMD_NOT_CLEARED   Reg 12 / Reg 23 tidak dikosongkan PLC > 1.5 detik
  WARN CONFIG_RANGE      Nilai konfigurasi di luar rentang valid
  WARN PAUSE_LONG        Pause aktif > --pause-alarm-min menit
  WARN PARADE_LONG       Parade lock aktif > --lock-alarm-min menit
Catatan: Reg 20 adalah output menurut program PLC, bukan kondisi lampu fisik (lampu putus tidak terdeteksi).
"""

import json
import time
import signal
import argparse
from datetime import datetime

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    try:
        from pymodbus.client.sync import ModbusTcpClient
    except ImportError:
        ModbusTcpClient = None

DEFAULT_PLC_IP    = "192.168.8.15"
DEFAULT_PLC_PORT  = 502
DEFAULT_SLAVE_ID  = 1
POLL_INTERVAL_S   = 1.0
STATUS_EVERY_S    = 1.0
RECONNECT_EVERY_S = 2.0
PLC_FAILSAFE_SEC  = 60    # Watchdog 2 di PLC (iMbIdleSec >= 60)

HEARTBEAT_STALE_S = 3.0
OVERTIME_MARGIN_S = 2
CMD_CLEAR_S       = 1.5

READ_COUNT = 24   # Reg 0..23 (PLC v0.7 / v0.7.1)
OUTPUT_BITS = ["traffic_green", "traffic_yellow", "traffic_red", "ped_green",
               "ped_red", None, "buzzer", "floodlight"]  # bit 5 (%Q0.5) SPARE

STATE_NAMES = {
    0: "Standby / Normal",
    1: "Traffic Standby (Tombol Ditekan)",
    2: "Traffic Yellow",
    3: "All Red 1 (Clearance)",
    4: "Pedestrian Crossing (Menyeberang)",
    5: "All Red 2 (Hitung Pelanggaran)",
    6: "Cooldown (Wait Before Normal)",
}

# Field konfigurasi yang perubahannya dicatat ([CFG] lama -> baru)
CONFIG_FIELDS = ["timers", "buzzer_cross", "default_mode", "parade_lock", "state_run",
                 "floodlight_mode", "floodlight_on", "floodlight_off"]


def to_signed_16(val: int) -> int:
    return val if val < 0x8000 else val - 0x10000


def read_registers(client, address, count, slave_id):
    """FC03, kompatibel pymodbus baru (device_id) dan lama (slave / unit)."""
    try:
        return client.read_holding_registers(address, count=count, device_id=slave_id)
    except TypeError:
        try:
            return client.read_holding_registers(address, count=count, slave=slave_id)
        except TypeError:
            return client.read_holding_registers(address, count=count, unit=slave_id)


def decode(regs):
    """Ubah list register mentah menjadi dict bernama."""
    return {
        "state": to_signed_16(regs[0]),
        "total_violations": to_signed_16(regs[1]),
        "timers": {f"state{i + 1}": to_signed_16(regs[2 + i]) for i in range(6)},
        "buzzer_cross": regs[8],
        "default_mode": regs[9],
        "parade_lock": regs[10],
        "state_run": regs[11],
        "manual_state": to_signed_16(regs[12]),
        "floodlight_mode": regs[13],
        "violation_side1": to_signed_16(regs[14]),
        "violation_side2": to_signed_16(regs[15]),
        "floodlight_on": f"{regs[16]:02d}:{regs[17]:02d}",
        "floodlight_off": f"{regs[18]:02d}:{regs[19]:02d}",
        "floodlight_raw": [to_signed_16(v) for v in regs[16:20]],
        "outputs": {n: bool(regs[20] & (1 << b)) for b, n in enumerate(OUTPUT_BITS) if n},
        "timer_elapsed": to_signed_16(regs[21]),
        "heartbeat": regs[22],
        "reset_violation": regs[23],
    }


def expected_outputs(state, default_mode):
    """Output yang WAJIB sesuai per state (mengikuti CASE di FB_PelicanCrossing).
    Kuning kedip (mode 1 di State 0/1) tidak diperiksa karena berganti tiap 0.5 detik."""
    ped_stop = {"ped_green": False, "ped_red": True}
    veh_red = {"traffic_green": False, "traffic_yellow": False, "traffic_red": True}
    veh_green = {"traffic_green": True, "traffic_yellow": False, "traffic_red": False}
    if state in (0, 1):
        veh = {"traffic_green": False, "traffic_red": False} if default_mode == 1 else veh_green
        return {**veh, **ped_stop}
    if state == 2:
        return {"traffic_green": False, "traffic_yellow": True, "traffic_red": False, **ped_stop}
    if state in (3, 5):
        return {**veh_red, **ped_stop}
    if state == 4:
        return {**veh_red, "ped_green": True, "ped_red": False}
    if state == 6:
        return {**veh_green, **ped_stop}
    return {}


def config_range_errors(d):
    errs = []
    for k, v in d["timers"].items():
        if not (1 <= v <= 3600):
            errs.append(f"{k}={v} (valid 1..3600)")
    if d["buzzer_cross"] not in (0, 1):
        errs.append(f"buzzer_cross={d['buzzer_cross']} (valid 0/1)")
    if d["default_mode"] not in (0, 1):
        errs.append(f"default_mode={d['default_mode']} (valid 0/1)")
    if d["parade_lock"] not in (0, 1):
        errs.append(f"parade_lock={d['parade_lock']} (valid 0/1)")
    if d["state_run"] not in (0, 1):
        errs.append(f"state_run={d['state_run']} (valid 0/1)")
    if d["floodlight_mode"] not in (0, 1, 2):
        errs.append(f"floodlight_mode={d['floodlight_mode']} (valid 0..2)")
    on_h, on_m, off_h, off_m = d["floodlight_raw"]
    if not (0 <= on_h <= 23 and 0 <= off_h <= 23 and 0 <= on_m <= 59 and 0 <= off_m <= 59):
        errs.append(f"jadwal lampu sorot {d['floodlight_raw']} (PLC memakai default 17:30-06:00)")
    return errs


class Monitor:
    """Menyimpan riwayat singkat pembacaan dan status alarm (aktif / normal).
    `on_event` (opsional): fungsi yang menerima dict event, dipakai mqtt_gateway.py untuk
    mengirim event ke broker. Tanpa `on_event`, perilaku sama persis (hanya print)."""

    def __init__(self, pause_alarm_s, lock_alarm_s, on_event=None):
        self.pause_alarm_s = pause_alarm_s
        self.lock_alarm_s = lock_alarm_s
        self.on_event = on_event
        self.reset()

    def emit(self, event):
        if self.on_event is not None:
            self.on_event(event)

    def reset(self):
        self.prev = None
        self.alarms = {}          # code -> {"since": t | None, "active": bool}
        self.hb_value = None
        self.hb_changed_at = None

    @staticmethod
    def log(level, code, msg):
        print(f"[{datetime.now():%H:%M:%S}] [{level}] {code}: {msg}")

    def check(self, code, cond, level, msg, now, hold=0.0):
        """Aktifkan alarm bila `cond` bertahan >= `hold` detik; catat saat kembali normal."""
        a = self.alarms.setdefault(code, {"since": None, "active": False})
        if cond:
            if a["since"] is None:
                a["since"] = now
            if not a["active"] and now - a["since"] >= hold:
                a["active"] = True
                self.log(level, code, msg)
                self.emit({"type": "alarm", "code": code, "level": level, "message": msg})
        else:
            if a["active"]:
                self.log("OK", code, "kembali normal")
                self.emit({"type": "alarm_clear", "code": code})
            a["since"] = None
            a["active"] = False

    def active_alarms(self):
        return [c for c, a in self.alarms.items() if a["active"]]

    def update(self, d, now):
        state = d["state"]
        out = d["outputs"]
        running = d["state_run"] == 1 and d["parade_lock"] == 0

        # --- Event: pergantian state, perubahan konfigurasi, reset pelanggaran ---
        if self.prev is not None:
            if state != self.prev["state"]:
                print(f"[STATE] {self.prev['state']} -> {state} ({STATE_NAMES.get(state, '?')})")
                self.emit({"type": "state_change", "from": self.prev["state"], "to": state,
                           "to_name": STATE_NAMES.get(state, "?")})
            for k in CONFIG_FIELDS:
                if d[k] != self.prev[k]:
                    print(f"[CFG] {k}: {self.prev[k]} -> {d[k]}")
                    self.emit({"type": "config_change", "field": k, "old": self.prev[k], "new": d[k]})
            if d["total_violations"] < self.prev["total_violations"]:
                print(f"[INFO] Pelanggaran direset: {self.prev['total_violations']} -> {d['total_violations']}")
                self.emit({"type": "violation_reset", "old_total": self.prev["total_violations"]})
            elif d["total_violations"] > self.prev["total_violations"]:
                delta = d["total_violations"] - self.prev["total_violations"]
                print(f"[INFO] Pelanggaran +{delta} "
                      f"| total {d['total_violations']} (S1 {d['violation_side1']}, S2 {d['violation_side2']})")
                self.emit({"type": "violation", "delta": delta, "total": d["total_violations"],
                           "side1": d["violation_side1"], "side2": d["violation_side2"]})

        # --- Heartbeat PLC ---
        if d["heartbeat"] != self.hb_value:
            self.hb_value = d["heartbeat"]
            self.hb_changed_at = now
        self.check("HEARTBEAT_STALE", now - self.hb_changed_at > HEARTBEAT_STALE_S, "CRIT",
                   f"heartbeat tetap {d['heartbeat']} > {HEARTBEAT_STALE_S:.0f} detik (program PLC macet / STOP?)", now)

        # --- Keselamatan output (Reg 20) ---
        conflicts = []
        if (out["traffic_green"] or out["traffic_yellow"]) and out["ped_green"]:
            conflicts.append("kendaraan hijau/kuning + pejalan kaki hijau")
        if out["ped_green"] and out["ped_red"]:
            conflicts.append("pejalan kaki hijau + merah")
        if out["traffic_green"] and out["traffic_red"]:
            conflicts.append("kendaraan hijau + merah")
        # Tanpa jeda: Reg 0 & Reg 20 ditulis di scan PLC yang sama, dan tidak ada transisi state
        # yang menghasilkan kombinasi ini -> satu pembacaan saja sudah berarti output bentrok.
        self.check("OUTPUT_CONFLICT", bool(conflicts), "CRIT",
                   f"{'; '.join(conflicts)} (state {state})", now)

        exp = expected_outputs(state, d["default_mode"])
        wrong = [f"{k}={int(out[k])} (harus {int(v)})" for k, v in exp.items() if out[k] != v]
        self.check("OUTPUT_MISMATCH", bool(wrong), "WARN",
                   f"state {state}: {', '.join(wrong)}", now, hold=1.0)

        # --- State machine ---
        self.check("STATE_INVALID", not (0 <= state <= 6), "WARN", f"state = {state}", now)
        dur = d["timers"].get(f"state{state}") if 1 <= state <= 6 else None
        overtime = running and dur is not None and d["timer_elapsed"] > dur + OVERTIME_MARGIN_S
        self.check("STATE_OVERTIME", overtime, "WARN",
                   f"state {state} sudah {d['timer_elapsed']} detik (durasi {dur} detik)", now)

        # --- Pelanggaran ---
        vsum = d["violation_side1"] + d["violation_side2"]
        self.check("VIOLATION_SUM", d["total_violations"] != vsum, "WARN",
                   f"total {d['total_violations']} != sisi1 + sisi2 = {vsum}", now, hold=0.5)

        # --- Perintah sekali-jalan harus dikosongkan PLC ---
        # PLC hanya memproses & mengosongkan Reg 12 bila >= 0 (nilai negatif = idle)
        self.check("CMD_NOT_CLEARED", d["manual_state"] >= 0 or d["reset_violation"] != 0, "WARN",
                   f"Reg 12 = {d['manual_state']}, Reg 23 = {d['reset_violation']} tidak dikosongkan PLC",
                   now, hold=CMD_CLEAR_S)

        # --- Konfigurasi ---
        errs = config_range_errors(d)
        self.check("CONFIG_RANGE", bool(errs), "WARN", "; ".join(errs), now)

        # --- Mode operasi berkepanjangan ---
        self.check("PAUSE_LONG", d["state_run"] == 0, "WARN",
                   f"PAUSE aktif > {self.pause_alarm_s / 60:.0f} menit di state {state}", now, hold=self.pause_alarm_s)
        self.check("PARADE_LONG", d["parade_lock"] != 0, "WARN",
                   f"Parade lock aktif > {self.lock_alarm_s / 60:.0f} menit", now, hold=self.lock_alarm_s)

        self.prev = d


def main():
    p = argparse.ArgumentParser(description="Pelican Crossing - Modbus TCP monitor (read-only)")
    p.add_argument("--plc-ip", default=DEFAULT_PLC_IP)
    p.add_argument("--plc-port", type=int, default=DEFAULT_PLC_PORT)
    p.add_argument("--slave-id", type=int, default=DEFAULT_SLAVE_ID)
    p.add_argument("--interval", type=float, default=POLL_INTERVAL_S, help="Interval polling detik (default 1.0)")
    p.add_argument("--pause-alarm-min", type=float, default=10, help="Alarm bila PAUSE aktif lebih dari N menit (default 10)")
    p.add_argument("--lock-alarm-min", type=float, default=60, help="Alarm bila Parade lock aktif lebih dari N menit (default 60)")
    p.add_argument("--once", action="store_true", help="Baca 1x, cetak seluruh data (JSON), lalu keluar")
    args = p.parse_args()

    if ModbusTcpClient is None:
        print("[!] pymodbus belum terinstall. Jalankan: pip3 install pymodbus")
        raise SystemExit(1)

    # systemd 'stop' (SIGTERM) -> keluar rapi agar soket Modbus ditutup
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    client = ModbusTcpClient(args.plc_ip, port=args.plc_port, timeout=0.5, retries=1)

    if args.once:
        try:
            if not client.connect():
                print(f"[!] Tidak dapat terhubung ke {args.plc_ip}:{args.plc_port}")
                raise SystemExit(1)
            resp = read_registers(client, 0, READ_COUNT, args.slave_id)
            if resp is None or resp.isError() or len(resp.registers) < READ_COUNT:
                print(f"[!] Gagal membaca Reg 0..{READ_COUNT - 1}: {resp}")
                raise SystemExit(1)
            print(json.dumps({"raw": list(resp.registers), **decode(list(resp.registers))}, indent=2))
        finally:
            client.close()
        return

    print(f"[*] Target PLC {args.plc_ip}:{args.plc_port} (slave {args.slave_id}) | read-only | polling {args.interval}s")
    monitor = Monitor(pause_alarm_s=args.pause_alarm_min * 60, lock_alarm_s=args.lock_alarm_min * 60)

    connected = False
    disconnected_since = None
    last_connect_try = 0.0
    last_status = 0.0

    def mark_disconnected(reason):
        nonlocal connected, disconnected_since
        if connected:
            print(f"[!] Koneksi PLC terputus: {reason}")
            disconnected_since = time.time()
            monitor.reset()
        connected = False

    while True:
        loop_start = time.time()
        try:
            if not client.is_socket_open():
                mark_disconnected("soket tertutup")
                if time.time() - last_connect_try >= RECONNECT_EVERY_S:
                    last_connect_try = time.time()
                    if client.connect():
                        print("[+] Terhubung ke PLC")
                    else:
                        print(f"[!] Tidak dapat terhubung ke {args.plc_ip}:{args.plc_port}, coba lagi {RECONNECT_EVERY_S:.0f}s...")

            if client.is_socket_open():
                resp = read_registers(client, 0, READ_COUNT, args.slave_id)
                if resp is not None and not resp.isError() and len(resp.registers) >= READ_COUNT:
                    now = time.time()
                    data = decode(list(resp.registers))
                    if not connected:
                        print(f"[+] Data PLC terbaca: State={data['state']}, Pelanggaran={data['total_violations']}")
                        if disconnected_since is not None and now - disconnected_since >= PLC_FAILSAFE_SEC:
                            print(f"[INFO] Putus {now - disconnected_since:.0f} detik (>= {PLC_FAILSAFE_SEC}s): fail-safe PLC "
                                  f"kemungkinan sudah melepas Parade lock / Pause. Saat ini lock={data['parade_lock']} "
                                  f"run={data['state_run']}")
                        disconnected_since = None
                    connected = True

                    monitor.update(data, now)

                    # Toleransi 0.1 s: jarak antar pembacaan bisa sedikit < 1 detik (jitter loop),
                    # tanpa toleransi sebagian baris status akan terlewat.
                    if now - last_status >= STATUS_EVERY_S - 0.1:
                        last_status = now
                        alarms = monitor.active_alarms()
                        print(f"[{datetime.now():%H:%M:%S}] S{data['state']} {STATE_NAMES.get(data['state'], '?')} | "
                              f"t={data['timer_elapsed']} | vio={data['total_violations']} "
                              f"(S1 {data['violation_side1']}, S2 {data['violation_side2']}) | "
                              f"run={data['state_run']} lock={data['parade_lock']} hb={data['heartbeat']} | "
                              f"alarm: {', '.join(alarms) if alarms else '-'}")
                else:
                    print(f"[!] Gagal membaca Reg 0..{READ_COUNT - 1} dari PLC: {resp} "
                          f"(pastikan program PLC v0.7 / v0.7.1, Holding_Data Array[0..29]) -> sambung ulang")
                    mark_disconnected("gagal membaca register")
                    client.close()

            time.sleep(max(0.0, args.interval - (time.time() - loop_start)))

        except KeyboardInterrupt:
            print("\n[*] Berhenti...")
            break
        except Exception as e:
            print(f"[!] Error loop: {e}")
            mark_disconnected(str(e))
            try:
                client.close()
            except Exception:
                pass
            time.sleep(1.0)

    client.close()   # FIN TCP -> MB_SERVER kembali listening
    print("[*] Koneksi Modbus ditutup.")


if __name__ == "__main__":
    main()
