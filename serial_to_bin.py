#!/usr/bin/env python3
"""pc_rx_save.py

Lê frames do ESP32 RX via serial (framing MAGIC + len + payload) e salva arquivos recebidos via LoRa.

IMPORTANTE (alinhado com o teu firmware do RX):
O RX NÃO encaminha o pacote LoRa bruto para o PC. Ele re-embala assim:

BEGIN (RX->PC):
  0x01
  tx_id (uint16 LE)
  img_id (uint32 LE)
  file_size (uint32 LE)
  chunk_size (uint16 LE)
  total_chunks (uint16 LE)
  name_len (uint8)
  name (name_len bytes, ASCII/UTF-8)

DATA (RX->PC):
  0x02
  img_id (uint32 LE)
  chunk_idx (uint16 LE)
  payload_len (uint16 LE)
  payload (payload_len bytes)
  rx_crc (uint16 LE)   # CRC do payload (o mesmo que veio do LoRa)

END (RX->PC):
  0x03
  img_id (uint32 LE)

LOG (RX->PC):
  0x10 + mlen(uint16) + msg(mlen)

HELLO (RX->PC):
  0x11 + millis(uint32)

PC->RX:
  Heartbeat frame (para liberar GRANTs): payload = [0x20]

Uso:
  python3 pc_rx_save.py /dev/ttyACM0 [out_dir] [--debug]
  python3 pc_rx_save.py --test
"""

from __future__ import annotations

import struct
import time
import sys
from pathlib import Path

import serial

# =========================
# Constantes de protocolo
# =========================

MAGIC = b"\xA5\x5A"

PKT_BEGIN = 0x01
PKT_DATA  = 0x02
PKT_END   = 0x03
PKT_LOG   = 0x10
PKT_HELLO = 0x11

SER_PKT_PC_HEARTBEAT = 0x20

# =========================
# Configurações
# =========================

MAX_FRAME = 4096            # proteção contra dessync
HB_PERIOD_S = 1.0           # heartbeat a cada 1s
PROG_MIN_INTERVAL_S = 0.08  # atualização da barra (~12x/s)


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT (polinômio 0x1021), init 0xFFFF, sem reflexão."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def safe_filename(name: str) -> str:
    name = (name or "").replace(" ", "").replace("\\", "/").split("/")[-1].strip()
    if not name:
        return ""
    name = "".join(c for c in name if 32 <= ord(c) < 127 and c not in "<>:\"|?*")
    return name[:128]


def fmt_eta(seconds: float | None) -> str:
    if seconds is None or seconds <= 0 or seconds == float("inf"):
        return "--:--"
    s = int(seconds)
    m, s = divmod(s, 60)
    h, m = divmod(m, 60)
    if h:
        return f"{h:d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def is_tty() -> bool:
    try:
        return sys.stdout.isatty()
    except Exception:
        return False


def render_progress(prefix: str, frac: float, cur: int, total: int,
                    speed_bps: float | None, eta_s: float | None,
                    width: int = 34):
    if total <= 0:
        return

    frac = min(max(frac, 0.0), 1.0)
    filled = int(frac * width)
    bar = "#" * filled + "-" * (width - filled)
    pct = int(frac * 100)

    spd = "?" if speed_bps is None else f"{speed_bps/1024:.1f} KB/s"
    eta = fmt_eta(eta_s)

    line = f"{prefix} [{bar}] {pct:3d}% ({cur}/{total})  {spd}  ETA {eta}"

    if is_tty():
        sys.stdout.write("" + line + "   ")
        sys.stdout.flush()
    else:
        # fallback: imprime de vez em quando
        if pct % 5 == 0 or pct == 100:
            print(line)


def read_exact(ser: serial.Serial, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            continue
        buf.extend(chunk)
    return bytes(buf)


def send_pc_frame(ser: serial.Serial, payload: bytes):
    """Framing PC->RX: MAGIC + len(uint16 LE) + payload."""
    ser.write(MAGIC)
    ser.write(struct.pack("<H", len(payload)))
    ser.write(payload)


def sync_to_magic(ser: serial.Serial, debug=False, on_idle=None):
    window = bytearray()
    last_report = time.time()
    seen = 0

    while True:
        b = ser.read(1)
        if b:
            seen += 1
            window += b
            if len(window) > 2:
                window = window[-2:]
            if bytes(window) == MAGIC:
                return
        else:
            if on_idle:
                on_idle()

        if debug and (time.time() - last_report > 2.0):
            last_report = time.time()
            if seen == 0:
                print("[DBG] Nenhum byte chegando da serial ainda...")
            else:
                print(f"[DBG] Recebidos {seen} bytes (ainda sem MAGIC)")


# =========================
# Parsers (RX->PC)
# =========================

def parse_begin_payload(payload: bytes):
    # 0x01 + tx_id(2) + img_id(4) + file_size(4) + chunk_size(2) + total_chunks(2) + name_len(1)
    header_len = 1 + 2 + 4 + 4 + 2 + 2 + 1
    if len(payload) < header_len:
        raise ValueError(f"BEGIN muito curto: len={len(payload)}")

    ptype = payload[0]
    if ptype != PKT_BEGIN:
        raise ValueError(f"ptype inválido no BEGIN: 0x{ptype:02X}")

    tx_id = struct.unpack_from("<H", payload, 1)[0]
    img_id = struct.unpack_from("<I", payload, 3)[0]
    file_size = struct.unpack_from("<I", payload, 7)[0]
    chunk_size = struct.unpack_from("<H", payload, 11)[0]
    total_chunks = struct.unpack_from("<H", payload, 13)[0]
    name_len = payload[15]

    if chunk_size == 0 or chunk_size > 4096:
        raise ValueError(f"BEGIN chunk_size inválido: {chunk_size}")
    if total_chunks == 0:
        raise ValueError("BEGIN total_chunks=0")

    name_len = int(name_len)
    if name_len < 0 or name_len > 64:
        # teu RX manda <=10, mas deixo margem
        name_len = min(max(name_len, 0), 10)

    if len(payload) < header_len + name_len:
        raise ValueError("BEGIN com nome incompleto")

    raw = payload[header_len:header_len + name_len]
    filename = raw.decode("utf-8", errors="replace").rstrip("_").strip()
    if filename:
        filename += ".bin"

    return tx_id, img_id, file_size, chunk_size, total_chunks, filename


def parse_data_payload(payload: bytes):
    # 0x02 + img_id(4) + chunk_idx(2) + payload_len(2) + payload + crc(2)
    header_len = 1 + 4 + 2 + 2
    if len(payload) < header_len + 2:
        raise ValueError(f"DATA muito curto: len={len(payload)}")

    ptype = payload[0]
    if ptype != PKT_DATA:
        raise ValueError(f"ptype inválido no DATA: 0x{ptype:02X}")

    img_id = struct.unpack_from("<I", payload, 1)[0]
    chunk_idx = struct.unpack_from("<H", payload, 5)[0]
    plen = struct.unpack_from("<H", payload, 7)[0]

    need = header_len + plen + 2
    if plen > 4096 or len(payload) < need:
        raise ValueError("DATA incompleto")

    data = payload[header_len:header_len + plen]
    rx_crc = struct.unpack_from("<H", payload, header_len + plen)[0]

    return img_id, chunk_idx, data, rx_crc


def parse_end_payload(payload: bytes):
    # 0x03 + img_id(4)
    if len(payload) < 1 + 4:
        raise ValueError(f"END muito curto: len={len(payload)}")
    ptype = payload[0]
    if ptype != PKT_END:
        raise ValueError(f"ptype inválido no END: 0x{ptype:02X}")
    img_id = struct.unpack_from("<I", payload, 1)[0]
    return img_id


# =========================
# Sessão de recepção
# =========================

class RxSession:
    def __init__(self, out_root: Path, debug: bool = False):
        self.out_root = out_root
        self.debug = debug
        self.close()

    def start(self, tx_id: int, img_id: int, file_size: int, chunk_size: int, total_chunks: int, filename: str | None):
        self.close()
        self.active = True
        self.tx_id = tx_id
        self.img_id = img_id
        self.file_size = file_size
        self.chunk_size = chunk_size
        self.total_chunks = total_chunks

        self.received = 0
        self.seen_chunks: set[int] = set()
        self._t0 = time.time()
        self._recv_bytes = 0
        self._last_draw = 0.0

        out_dir = self.out_root / f"TX{tx_id:04d}"
        out_dir.mkdir(parents=True, exist_ok=True)

        fname = safe_filename(filename or "")
        if not fname:
            fname = f"IMG_{img_id:08X}.bin"

        self.path = out_dir / fname
        self.f = open(self.path, "wb")
        try:
            self.f.truncate(file_size)
        except Exception:
            pass

        print(f"[BEGIN] TX{tx_id:04d} img={img_id:08X} size={file_size} chunk={chunk_size} total={total_chunks}")
        print(f"[OUT] {self.path}")
        self._draw(force=True)

    def _draw(self, force: bool = False):
        if not self.active or self.total_chunks <= 0:
            return

        now = time.time()
        if not force and (now - self._last_draw) < PROG_MIN_INTERVAL_S:
            return
        self._last_draw = now

        frac = self.received / self.total_chunks
        dt = max(now - self._t0, 1e-6)
        speed = self._recv_bytes / dt
        remaining = (self.total_chunks - self.received) * self.chunk_size
        eta = remaining / speed if speed > 1e-3 else None

        render_progress(
            prefix=f"[RX] TX{self.tx_id:04d} IMG{self.img_id:08X}",
            frac=frac,
            cur=self.received,
            total=self.total_chunks,
            speed_bps=speed,
            eta_s=eta,
        )

    def write_chunk(self, img_id: int, chunk_idx: int, payload: bytes, rx_crc: int):
        if not self.active:
            return
        if img_id != self.img_id:
            return
        if chunk_idx >= self.total_chunks:
            return
        if chunk_idx in self.seen_chunks:
            self._draw()
            return

        calc = crc16_ccitt(payload)
        if calc != rx_crc:
            if is_tty():
                sys.stdout.write("")
                sys.stdout.flush()
            print(f"[WARN] CRC errado no chunk {chunk_idx} (calc={calc:04X} rx={rx_crc:04X})")
            return

        self.f.seek(chunk_idx * self.chunk_size)
        self.f.write(payload)

        self.seen_chunks.add(chunk_idx)
        self.received += 1
        self._recv_bytes += len(payload)
        self._draw()

    def finish(self, img_id: int):
        if not self.active:
            return
        if img_id != self.img_id:
            return

        try:
            self.f.flush()
            self.f.close()
        finally:
            self.f = None

        self._draw(force=True)
        if is_tty():
            sys.stdout.write("")
            sys.stdout.flush()

        missing = self.total_chunks - self.received
        elapsed = max(time.time() - self._t0, 1e-6)
        avg = self._recv_bytes / elapsed

        if missing:
            print(f"[DONE] {self.path} ({self.file_size} bytes) faltaram {missing} chunks avg {avg/1024:.1f} KB/s")
        else:
            print(f"[DONE] {self.path} ({self.file_size} bytes) avg {avg/1024:.1f} KB/s")

        self.active = False

    def close(self):
        if getattr(self, "f", None):
            try:
                self.f.close()
            except Exception:
                pass
        self.active = False
        self.tx_id = None
        self.img_id = None
        self.file_size = None
        self.chunk_size = None
        self.total_chunks = None
        self.received = 0
        self.f = None
        self.path = None


# =========================
# Tests
# =========================

def _run_tests():
    tx_id = 1
    img_id = 0x12345678
    file_size = 10804
    chunk_size = 200
    total_chunks = 55
    name = b"IMG_0001__"  # 10 bytes
    name_len = len(name)

    begin = struct.pack("<B H I I H H B", PKT_BEGIN, tx_id, img_id, file_size, chunk_size, total_chunks, name_len) + name
    out = parse_begin_payload(begin)
    assert out[0] == tx_id and out[1] == img_id and out[2] == file_size and out[3] == chunk_size and out[4] == total_chunks

    payload = b"abcd" * 10
    plen = len(payload)
    rx_crc = crc16_ccitt(payload)
    data = struct.pack("<B I H H", PKT_DATA, img_id, 3, plen) + payload + struct.pack("<H", rx_crc)
    out2 = parse_data_payload(data)
    assert out2[0] == img_id and out2[1] == 3 and out2[2] == payload and out2[3] == rx_crc

    end = struct.pack("<B I", PKT_END, img_id)
    out3 = parse_end_payload(end)
    assert out3 == img_id

    print("[TEST] OK")


# =========================
# Main
# =========================

def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--test":
        _run_tests()
        return

    if len(sys.argv) < 2:
        print("Uso: python3 pc_rx_save.py /dev/ttyACM0 [out_dir] [--debug]")
        print("     python3 pc_rx_save.py --test")
        raise SystemExit(1)

    port = sys.argv[1]
    out_dir = Path(sys.argv[2]) if len(sys.argv) >= 3 and not sys.argv[2].startswith("--") else Path("thumb_bin")
    debug = ("--debug" in sys.argv)

    ser = serial.Serial(port, 115200, timeout=0.1)
    sess = RxSession(out_dir, debug=debug)

    print(f"[OK] Lendo frames binários em {port} (115200)")
    print("[INFO] Heartbeat ON; aguardando BEGIN...")

    last_hb = 0.0

    def maybe_hb():
        nonlocal last_hb
        now = time.time()
        if now - last_hb >= HB_PERIOD_S:
            send_pc_frame(ser, bytes([SER_PKT_PC_HEARTBEAT]))
            last_hb = now

    try:
        while True:
            maybe_hb()
            sync_to_magic(ser, debug=debug, on_idle=maybe_hb)

            ln = struct.unpack("<H", read_exact(ser, 2))[0]
            if ln == 0 or ln > MAX_FRAME:
                continue

            payload = read_exact(ser, ln)
            if not payload:
                continue

            ptype = payload[0]

            if ptype == PKT_HELLO:
                if debug and len(payload) >= 5:
                    t = struct.unpack_from("<I", payload, 1)[0]
                    print(f"[HELLO] t={t}ms")
                continue

            if ptype == PKT_LOG:
                if len(payload) >= 3:
                    mlen = struct.unpack_from("<H", payload, 1)[0]
                    msg = payload[3:3+mlen].decode("utf-8", errors="replace")
                    if sess.active and is_tty():
                        sys.stdout.write("")
                        sys.stdout.flush()
                    print(msg)
                continue

            if ptype == PKT_BEGIN:
                try:
                    tx_id, img_id, fs, cs, tc, fn = parse_begin_payload(payload)
                except Exception as e:
                    if debug:
                        print(f"[DBG] BEGIN inválido: {e}")
                        print(f"[DBG] BEGIN len={len(payload)} bytes: {payload.hex(' ')}")
                    continue
                sess.start(tx_id, img_id, fs, cs, tc, fn)
                continue

            if ptype == PKT_DATA:
                try:
                    img_id, idx, data, rx_crc = parse_data_payload(payload)
                except Exception as e:
                    if debug:
                        print(f"[DBG] DATA inválido: {e}")
                        print(f"[DBG] DATA len={len(payload)} bytes")
                    continue
                sess.write_chunk(img_id, idx, data, rx_crc)
                continue

            if ptype == PKT_END:
                try:
                    img_id = parse_end_payload(payload)
                except Exception as e:
                    if debug:
                        print(f"[DBG] END inválido: {e}")
                        print(f"[DBG] END len={len(payload)} bytes")
                    continue
                sess.finish(img_id)
                continue

            if debug:
                print(f"[DBG] Frame desconhecido type=0x{ptype:02X} len={ln}")

    except KeyboardInterrupt:
        if is_tty():
            sys.stdout.write("")
            sys.stdout.flush()
        print("[SAINDO]")
    finally:
        sess.close()
        ser.close()


if __name__ == "__main__":
    main()
