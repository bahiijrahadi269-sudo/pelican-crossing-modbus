"""
Pelican Crossing - Terminal Modbus TCP (BACA + TULIS)
Jetson Orin Nano  --Modbus TCP-->  Siemens S7-1200 (MB_SERVER, port 502)

Memantau PLC seperti modbus_monitor.py (pembacaan, alarm, log state/konfigurasi) DAN menerima
perintah tulis yang diketik langsung di terminal. Semua akses Modbus memakai SATU koneksi,
karena PLC hanya menerima satu klien (CONNECT_ID 1).
-> Jalankan program ini SEBAGAI PENGGANTI modbus_monitor.py, jangan keduanya bersamaan.

Membutuhkan modbus_monitor.py di folder yang sama (fungsi baca, decode, dan alarm dipakai ulang).
Satu-satunya dependensi non-stdlib: pymodbus.

Keamanan penulisan:
  - Hanya register parameter/perintah yang boleh ditulis: 2..13, 16..19, 23.
    Register status (0, 1, 14, 15, 20, 21, 22) ditolak.
  - Setiap nilai divalidasi rentangnya sebelum dikirim.
  - Perintah berisiko (parade lock, pause, lompat state, reset pelanggaran, tulis mentah)
    harus dikonfirmasi dengan mengetik 'ya' dalam 20 detik.
  - Setelah menulis, register dibaca ulang; perintah dianggap berhasil hanya bila nilainya
    benar-benar berubah di PLC (Reg 12 / Reg 23: bila sudah dikosongkan PLC).
  - Lompat state (Reg 12) sebaiknya dipakai dengan program PLC v0.7.1 (interlock lengkap).

Ketik 'help' saat program berjalan untuk daftar perintah.
"""

import sys
import json
import time
import queue
import signal
import argparse
import threading
from datetime import datetime

import modbus_monitor as mm

WRITABLE_REGS = set(range(2, 14)) | set(range(16, 20)) | {23}
CONFIRM_TIMEOUT_S = 20
READBACK_DELAY_S = 0.1

HELP_TEXT = """
Perintah BACA:
  read                  Baca semua register sekarang dan tampilkan (JSON)
  read <reg>            Baca satu register, misal: read 11
  status on|off         Nyalakan / matikan baris status berkala (alarm & event tetap tampil)

Perintah TULIS:
  timer <1-6> <detik>   Durasi satu state (1..3600 detik)        misal: timer 4 25
  timers <s1> .. <s6>   Durasi State 1..6 sekaligus               misal: timers 10 3 3 20 3 60
  buzzer on|off         Buzzer saat State 4 (Reg 8)
  mode hijau|kedip      Mode standby (Reg 9)
  sorot auto|on|off     Lampu sorot (Reg 13)
  jadwal HH:MM HH:MM    Jadwal lampu sorot ON dan OFF (Reg 16..19) misal: jadwal 17:30 06:00
  run                   Jalankan FSM (Reg 11 = 1)
  pause                 Pause FSM (Reg 11 = 0)                    [konfirmasi]
  parade on|off         Parade lock (Reg 10)                      [konfirmasi]
  lompat <0-6>          Lompat state manual (Reg 12)              [konfirmasi]
  reset                 Reset akumulasi pelanggaran (Reg 23 = 1)  [konfirmasi]
  write <reg> <nilai> [nilai ...]
                        Tulis mentah ke register berurutan mulai <reg>
                        (hanya Reg 2..13, 16..19, 23)             [konfirmasi]

Lainnya:
  help                  Tampilkan bantuan ini
  quit                  Keluar (koneksi Modbus ditutup rapi)
"""


def to_unsigned_16(val):
    """Nilai signed (mis. -1) -> format register unsigned 16-bit yang dikirim pymodbus."""
    return int(val) & 0xFFFF


def write_values(client, address, values, slave_id):
    """FC06 (1 nilai) atau FC16 (beberapa nilai), kompatibel pymodbus baru dan lama."""
    vals = [to_unsigned_16(v) for v in values]
    if len(vals) == 1:
        fn, kwargs = client.write_register, {"value": vals[0]}
    else:
        fn, kwargs = client.write_registers, {"values": vals}
    for id_kw in ("device_id", "slave", "unit"):
        try:
            return fn(address, **kwargs, **{id_kw: slave_id})
        except TypeError:
            continue
    raise RuntimeError("Versi pymodbus tidak dikenali (write_register / write_registers)")


def validate(reg, val):
    """Kembalikan pesan error bila register/nilai tidak boleh ditulis, None bila valid."""
    if reg not in WRITABLE_REGS:
        return f"Reg {reg} tidak boleh ditulis (hanya Reg 2..13, 16..19, 23)"
    if 2 <= reg <= 7:
        ok = 1 <= val <= 3600
        rng = "1..3600 detik"
    elif reg in (8, 9, 10, 11):
        ok, rng = val in (0, 1), "0 atau 1"
    elif reg == 12:
        ok, rng = 0 <= val <= 6, "0..6"
    elif reg == 13:
        ok, rng = val in (0, 1, 2), "0, 1, atau 2"
    elif reg in (16, 18):
        ok, rng = 0 <= val <= 23, "jam 0..23"
    elif reg in (17, 19):
        ok, rng = 0 <= val <= 59, "menit 0..59"
    else:  # reg 23
        ok, rng = val == 1, "hanya 1"
    return None if ok else f"Nilai Reg {reg} = {val} di luar rentang ({rng})"


def parse_hhmm(text):
    h, m = [int(x) for x in text.split(":")]
    return h, m


def build_plan(parts):
    """Ubah perintah tulis menjadi rencana {desc, address, values, confirm}.
    Kembalikan (plan, None) atau (None, pesan_error)."""
    cmd, args = parts[0].lower(), parts[1:]
    try:
        if cmd == "timer" and len(args) == 2:
            n, d = int(args[0]), int(args[1])
            if not 1 <= n <= 6:
                return None, "Nomor state harus 1..6"
            plan = {"desc": f"Durasi State {n} = {d} detik", "address": 1 + n, "values": [d], "confirm": False}
        elif cmd == "timers" and len(args) == 6:
            vals = [int(a) for a in args]
            plan = {"desc": f"Durasi State 1..6 = {vals} detik", "address": 2, "values": vals, "confirm": False}
        elif cmd == "buzzer" and len(args) == 1 and args[0].lower() in ("on", "off"):
            v = 1 if args[0].lower() == "on" else 0
            plan = {"desc": f"Buzzer State 4 = {args[0].upper()}", "address": 8, "values": [v], "confirm": False}
        elif cmd == "mode" and len(args) == 1 and args[0].lower() in ("hijau", "kedip"):
            v = 1 if args[0].lower() == "kedip" else 0
            plan = {"desc": f"Mode standby = {args[0].lower()}", "address": 9, "values": [v], "confirm": False}
        elif cmd == "sorot" and len(args) == 1 and args[0].lower() in ("auto", "on", "off"):
            v = {"auto": 0, "on": 1, "off": 2}[args[0].lower()]
            plan = {"desc": f"Lampu sorot = {args[0].upper()}", "address": 13, "values": [v], "confirm": False}
        elif cmd == "jadwal" and len(args) == 2:
            on_h, on_m = parse_hhmm(args[0])
            off_h, off_m = parse_hhmm(args[1])
            plan = {"desc": f"Jadwal lampu sorot ON {on_h:02d}:{on_m:02d}, OFF {off_h:02d}:{off_m:02d}",
                    "address": 16, "values": [on_h, on_m, off_h, off_m], "confirm": False}
        elif cmd == "run" and not args:
            plan = {"desc": "FSM RUN (Reg 11 = 1)", "address": 11, "values": [1], "confirm": False}
        elif cmd == "pause" and not args:
            plan = {"desc": "FSM PAUSE (Reg 11 = 0): lampu berhenti di state saat ini",
                    "address": 11, "values": [0], "confirm": True}
        elif cmd == "parade" and len(args) == 1 and args[0].lower() in ("on", "off"):
            v = 1 if args[0].lower() == "on" else 0
            desc = ("PARADE LOCK ON: paksa State 0, tombol diabaikan" if v else "PARADE LOCK OFF")
            plan = {"desc": desc, "address": 10, "values": [v], "confirm": True}
        elif cmd == "lompat" and len(args) == 1:
            n = int(args[0])
            plan = {"desc": f"Lompat ke State {n} ({mm.STATE_NAMES.get(n, '?')}); interlock PLC bisa mengalihkan lewat state lain",
                    "address": 12, "values": [n], "confirm": True}
        elif cmd == "reset" and not args:
            plan = {"desc": "RESET akumulasi pelanggaran (Reg 1, 14, 15 menjadi 0)",
                    "address": 23, "values": [1], "confirm": True}
        elif cmd == "write" and len(args) >= 2:
            reg, vals = int(args[0]), [int(a) for a in args[1:]]
            plan = {"desc": f"Tulis mentah Reg {reg}..{reg + len(vals) - 1} = {vals}",
                    "address": reg, "values": vals, "confirm": True}
        else:
            return None, f"Perintah tidak dikenal / format salah: '{' '.join(parts)}'. Ketik 'help'."
    except ValueError:
        return None, f"Angka atau format jam tidak valid: '{' '.join(parts)}'"

    for i, v in enumerate(plan["values"]):
        err = validate(plan["address"] + i, v)
        if err:
            return None, err
    return plan, None


def read_snapshot(client, slave_id):
    """Baca Reg 0..23; kembalikan list nilai signed atau None bila gagal."""
    resp = mm.read_registers(client, 0, mm.READ_COUNT, slave_id)
    if resp is None or resp.isError() or len(resp.registers) < mm.READ_COUNT:
        return None
    return list(resp.registers)


def apply_plan(client, slave_id, plan, last_regs):
    """Kirim penulisan, lalu baca ulang untuk memastikan PLC benar-benar berubah.
    Kembalikan dict {ok, code, detail, regs, state}; code = OK | WRITE_FAILED | NOT_CONFIRMED
    (sama dengan kode balasan di kontrak MQTT)."""
    addr, vals = plan["address"], plan["values"]
    old = [mm.to_signed_16(last_regs[addr + i]) for i in range(len(vals))] if last_regs else None

    resp = write_values(client, addr, vals, slave_id)
    if resp is None or (hasattr(resp, "isError") and resp.isError()):
        return {"ok": False, "code": "WRITE_FAILED", "detail": f"PLC menolak / tidak menjawab: {resp}",
                "regs": None, "state": None}

    time.sleep(READBACK_DELAY_S)
    regs = read_snapshot(client, slave_id)
    if regs is None:
        return {"ok": False, "code": "NOT_CONFIRMED", "detail": "terkirim, tetapi pembacaan ulang gagal",
                "regs": None, "state": None}
    now_vals = [mm.to_signed_16(regs[addr + i]) for i in range(len(vals))]
    state = mm.to_signed_16(regs[0])

    if addr == 12:
        # Perintah sekali-jalan: PLC mengosongkan Reg 12 ke -1 setelah memprosesnya
        ok = now_vals[0] == -1
        detail = (f"diproses PLC, state sekarang {state} ({mm.STATE_NAMES.get(state, '?')})" if ok
                  else f"Reg 12 masih {now_vals[0]} (belum diproses PLC)")
    elif addr == 23:
        ok = now_vals[0] == 0 and mm.to_signed_16(regs[1]) == 0
        detail = ("diproses PLC, total pelanggaran = 0" if ok
                  else f"Reg 23 = {now_vals[0]}, total pelanggaran = {mm.to_signed_16(regs[1])}")
    else:
        ok = now_vals == vals
        detail = f"lama {old} -> sekarang {now_vals}" if old is not None else f"sekarang {now_vals}"

    return {"ok": ok, "code": "OK" if ok else "NOT_CONFIRMED", "detail": detail, "regs": regs, "state": state}


def print_result(plan, res, source="terminal"):
    tag = {"OK": "WRITE OK", "WRITE_FAILED": "WRITE GAGAL"}.get(res["code"], "WRITE TIDAK SESUAI")
    print(f"[{datetime.now():%H:%M:%S}] [{tag}] ({source}) {plan['desc']} | {res['detail']}")


class Link:
    """Koneksi Modbus bersama (satu-satunya ke PLC) + snapshot register terakhir."""

    def __init__(self, client, slave_id):
        self.client = client
        self.slave_id = slave_id
        self.connected = False
        self.last_regs = None

    def run_plan(self, plan, source="terminal"):
        res = apply_plan(self.client, self.slave_id, plan, self.last_regs)
        if res["regs"] is not None:
            self.last_regs = res["regs"]
        print_result(plan, res, source)
        return res


class Console:
    """Perintah yang diketik di terminal. Dipakai modbus_terminal.py dan mqtt_gateway.py."""

    def __init__(self, link):
        self.link = link
        self.pending = None          # rencana tulis yang menunggu konfirmasi 'ya'
        self.pending_until = 0.0
        self.status_on = True
        self.quit_requested = False

    def expire_pending(self):
        if self.pending is not None and time.time() > self.pending_until:
            self.pending = None
            print("[*] Konfirmasi habis waktu, perintah dibatalkan.")

    def handle(self, line):
        """Proses satu baris perintah (dipanggil di thread utama)."""
        link = self.link
        if not line:
            return
        if self.pending is not None:
            plan, self.pending = self.pending, None
            if line.lower() in ("ya", "y", "yes"):
                if link.connected:
                    link.run_plan(plan)
                else:
                    print("[!] Dibatalkan: belum terhubung ke PLC")
            else:
                print("[*] Dibatalkan.")
            return

        parts = line.split()
        cmd = parts[0].lower()
        if cmd in ("help", "?"):
            print(HELP_TEXT)
        elif cmd in ("quit", "exit", "keluar"):
            self.quit_requested = True
        elif cmd == "status" and len(parts) == 2 and parts[1].lower() in ("on", "off"):
            self.status_on = parts[1].lower() == "on"
            print(f"[*] Baris status berkala {'ON' if self.status_on else 'OFF'}")
        elif cmd == "read":
            if not link.connected:
                print("[!] Belum terhubung ke PLC")
                return
            regs = read_snapshot(link.client, link.slave_id)
            if regs is None:
                print("[!] Gagal membaca PLC")
                return
            link.last_regs = regs
            if len(parts) == 1:
                print(json.dumps(mm.decode(regs), indent=2))
            else:
                try:
                    reg = int(parts[1])
                except ValueError:
                    print("[!] Nomor register tidak valid")
                    return
                if not 0 <= reg < mm.READ_COUNT:
                    print(f"[!] Register harus 0..{mm.READ_COUNT - 1}")
                    return
                print(f"Reg {reg} (4{reg + 1:04d}) = {mm.to_signed_16(regs[reg])}")
        else:
            plan, err = build_plan(parts)
            if err:
                print(f"[!] {err}")
                return
            if not link.connected:
                print("[!] Belum terhubung ke PLC, perintah tidak dikirim")
                return
            if plan["confirm"]:
                self.pending, self.pending_until = plan, time.time() + CONFIRM_TIMEOUT_S
                print(f"[?] {plan['desc']}")
                print(f"    Ketik 'ya' untuk mengirim ke PLC (batal otomatis dalam {CONFIRM_TIMEOUT_S} detik)")
            else:
                link.run_plan(plan)


def print_status_line(data, alarms):
    print(f"[{datetime.now():%H:%M:%S}] S{data['state']} {mm.STATE_NAMES.get(data['state'], '?')} | "
          f"t={data['timer_elapsed']} | vio={data['total_violations']} "
          f"(S1 {data['violation_side1']}, S2 {data['violation_side2']}) | "
          f"run={data['state_run']} lock={data['parade_lock']} hb={data['heartbeat']} | "
          f"alarm: {', '.join(alarms) if alarms else '-'}")


def stdin_reader(q):
    """Thread pembaca keyboard: hanya memasukkan baris ke antrean.
    Semua akses Modbus tetap di thread utama (klien pymodbus sinkron tidak thread-safe)."""
    try:
        for line in sys.stdin:
            q.put(line.strip())
    except Exception:
        pass   # stdin tidak tersedia (mis. dijalankan sebagai service): program tetap memantau


def main():
    p = argparse.ArgumentParser(description="Pelican Crossing - terminal Modbus TCP (baca + tulis)")
    p.add_argument("--plc-ip", default=mm.DEFAULT_PLC_IP)
    p.add_argument("--plc-port", type=int, default=mm.DEFAULT_PLC_PORT)
    p.add_argument("--slave-id", type=int, default=mm.DEFAULT_SLAVE_ID)
    p.add_argument("--interval", type=float, default=mm.POLL_INTERVAL_S, help="Interval polling detik (default 1.0)")
    p.add_argument("--pause-alarm-min", type=float, default=10, help="Alarm bila PAUSE aktif lebih dari N menit (default 10)")
    p.add_argument("--lock-alarm-min", type=float, default=60, help="Alarm bila Parade lock aktif lebih dari N menit (default 60)")
    args = p.parse_args()

    if mm.ModbusTcpClient is None:
        print("[!] pymodbus belum terinstall. Jalankan: pip3 install pymodbus")
        raise SystemExit(1)

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    client = mm.ModbusTcpClient(args.plc_ip, port=args.plc_port, timeout=0.5, retries=1)
    print(f"[*] Target PLC {args.plc_ip}:{args.plc_port} (slave {args.slave_id}) | BACA + TULIS | polling {args.interval}s")
    print("[*] Pastikan tidak ada program Modbus lain yang tersambung ke PLC (PLC hanya menerima 1 koneksi).")
    print("[*] Ketik 'help' lalu Enter untuk daftar perintah.")

    monitor = mm.Monitor(pause_alarm_s=args.pause_alarm_min * 60, lock_alarm_s=args.lock_alarm_min * 60)
    link = Link(client, args.slave_id)
    console = Console(link)
    cmd_q = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmd_q,), daemon=True).start()

    disconnected_since = None
    last_connect_try = 0.0
    last_status = 0.0

    def mark_disconnected(reason):
        nonlocal disconnected_since
        if link.connected:
            print(f"[!] Koneksi PLC terputus: {reason}")
            disconnected_since = time.time()
            monitor.reset()
        link.connected = False

    while not console.quit_requested:
        loop_start = time.time()
        try:
            # ---------------- (1) KONEKSI + BACA + ALARM (sama seperti modbus_monitor) ----------------
            if not client.is_socket_open():
                mark_disconnected("soket tertutup")
                if time.time() - last_connect_try >= mm.RECONNECT_EVERY_S:
                    last_connect_try = time.time()
                    if client.connect():
                        print("[+] Terhubung ke PLC")
                    else:
                        print(f"[!] Tidak dapat terhubung ke {args.plc_ip}:{args.plc_port}, coba lagi {mm.RECONNECT_EVERY_S:.0f}s...")

            if client.is_socket_open():
                regs = read_snapshot(client, args.slave_id)
                if regs is not None:
                    now = time.time()
                    link.last_regs = regs
                    data = mm.decode(regs)
                    if not link.connected:
                        print(f"[+] Data PLC terbaca: State={data['state']}, Pelanggaran={data['total_violations']}")
                        if disconnected_since is not None and now - disconnected_since >= mm.PLC_FAILSAFE_SEC:
                            print(f"[INFO] Putus {now - disconnected_since:.0f} detik (>= {mm.PLC_FAILSAFE_SEC}s): fail-safe PLC "
                                  f"kemungkinan sudah melepas Parade lock / Pause. Saat ini lock={data['parade_lock']} "
                                  f"run={data['state_run']}")
                        disconnected_since = None
                    link.connected = True

                    monitor.update(data, now)

                    if console.status_on and now - last_status >= mm.STATUS_EVERY_S - 0.1:
                        last_status = now
                        print_status_line(data, monitor.active_alarms())
                else:
                    print(f"[!] Gagal membaca Reg 0..{mm.READ_COUNT - 1} dari PLC -> sambung ulang")
                    mark_disconnected("gagal membaca register")
                    client.close()

            # ---------------- (2) PERINTAH TERMINAL sampai jadwal polling berikutnya ----------------
            deadline = loop_start + args.interval
            while not console.quit_requested:
                console.expire_pending()
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                try:
                    line = cmd_q.get(timeout=remaining)
                except queue.Empty:
                    break
                console.handle(line)

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
