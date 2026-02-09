#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv
import time
import socket
import json
from typing import Dict, Tuple, Optional, Any, List
from collections import deque

import numpy as np
import serial

from datetime import datetime
from zoneinfo import ZoneInfo


# =========================================================
# 設定
# =========================================================

# --- シリアル（入力：UWBログ） ---
SERIAL_PORT_IN = "/dev/ttyAMA0"
BAUDRATE_IN = 115200
TIMEOUT_IN = 0.2

# --- シリアル（出力：ArduPilotへ送るUART） ---
SERIAL_PORT_OUT = "/dev/ttySC0"
BAUDRATE_OUT = 115200
TIMEOUT_OUT = 0.1

# DWMに送るコマンド
INIT_COMMAND = "INITF\r\n"
STOP_COMMAND = "STOP\r\n"
AFTER_INIT_SLEEP_SEC = 0.3

# --- UDP送信先（GUIサーバ） ---
UDP_HOST = "192.168.2.101" #随時変更
UDP_PORT = 9900

# --- アンカー座標（m）mac_addressと対応させる
ANCHORS: Dict[str, Tuple[float, float, float]] = {
    "0x0001": (0.0, 0.0, 2.0),
    "0x0002": (3.0, 0.0, 0.0),
    "0x0003": (0.0, 3.0, 0.0),
    "0x0004": (3.0, 3.0, 2.0),
}

# beaconの順序（固定）
ANCHOR_ORDER: List[str] = ["0x0001", "0x0002", "0x0003", "0x0004"]
NUM_ANCHORS = len(ANCHOR_ORDER)

# --- 推定（Gauss-Newton） ---
MAX_ITERS = 30
TOL = 1e-6

# --- ログ ---
LOG_DIR = "./logs"
os.makedirs(LOG_DIR, exist_ok=True)
LOG_PATH = os.path.join(LOG_DIR, f"uwb_to_ardupilot_{time.strftime('%Y%m%d_%H%M%S')}.csv")

# --- ブロック判定 ---
SESSION_START = "SESSION_INFO_NTF:"
SESSION_END = "]}"

# =========================================================
# 送信・平滑化パラメータ
# =========================================================
SEND_HZ = 20.0
SEND_INTERVAL_SEC = 1.0 / SEND_HZ  # 0.05s

SMOOTH_WINDOW_SEC = 0.5          # 平均窓（直近500ms）
SMOOTH_UPDATE_SEC = 0.1          # 平滑化値の更新周期（100msに修正推奨。送信頻度に近いほうが滑らかです）
                                 # ※ 元のコードは0.5sでしたが、0.1sの方が追従性が良いです。

# =========================================================
# 緯度経度変換（ローカルENU -> WGS84近似）
# =========================================================
ORIGIN_LAT_DEG = 34.981650000
ORIGIN_LON_DEG = 135.964250000
ORIGIN_ALT_M = 149.921

# X軸(正)が西なので、東へはマイナス方向 (-1.0)
AXIS_X_TO_EAST = -1.0
# Y軸(正)が南なので、北へはマイナス方向 (-1.0)
AXIS_Y_TO_NORTH = -1.0
AXIS_Z_TO_UP = 1.0

EARTH_RADIUS_M = 6378137.0

def enu_to_latlon_alt(
    x_m: float,
    y_m: float,
    z_m: float,
    lat0_deg: float = ORIGIN_LAT_DEG,
    lon0_deg: float = ORIGIN_LON_DEG,
    alt0_m: float = ORIGIN_ALT_M
) -> Tuple[float, float, float]:
    east = AXIS_X_TO_EAST * x_m
    north = AXIS_Y_TO_NORTH * y_m
    up = AXIS_Z_TO_UP * z_m

    lat0 = np.deg2rad(lat0_deg)
    lon0 = np.deg2rad(lon0_deg)

    dlat = north / EARTH_RADIUS_M
    dlon = east / (EARTH_RADIUS_M * np.cos(lat0) + 1e-12)

    lat = lat0 + dlat
    lon = lon0 + dlon
    alt = alt0_m + up

    return float(np.rad2deg(lat)), float(np.rad2deg(lon)), float(alt)

# =========================================================
# パーサ
# =========================================================
MEAS_BLOCK_RE = re.compile(r"\[(.*?)\]\s*(?:;|\}\s*)", re.DOTALL)
MAC_RE = re.compile(r"mac_address\s*=\s*(0x[0-9a-fA-F]+)")
STATUS_RE = re.compile(r"status\s*=\s*\"([A-Z_]+)\"")
DIST_RE = re.compile(r"distance\[cm\]\s*=\s*([0-9]+)")
SEQ_RE = re.compile(r"sequence_number\s*=\s*([0-9]+)")
NMEAS_RE = re.compile(r"n_measurements\s*=\s*([0-9]+)")

def parse_session(block: str) -> Dict[str, Any]:
    seq = None
    n_meas = None

    m = SEQ_RE.search(block)
    if m:
        seq = int(m.group(1))

    m = NMEAS_RE.search(block)
    if m:
        n_meas = int(m.group(1))

    ranges_m: Dict[str, float] = {}
    for mb in MEAS_BLOCK_RE.finditer(block):
        inner = mb.group(1)
        mm = MAC_RE.search(inner)
        sm = STATUS_RE.search(inner)
        dm = DIST_RE.search(inner)
        if not mm or not sm or not dm:
            continue
        mac = mm.group(1).lower()
        status = sm.group(1)
        dist_cm = int(dm.group(1))
        if status != "SUCCESS":
            continue
        ranges_m[mac] = dist_cm / 100.0

    return {
        "sequence_number": seq,
        "n_measurements": n_meas,
        "ranges_m": ranges_m,
    }

def read_session_block(ser: serial.Serial) -> Optional[str]:
    buf_lines: List[str] = []
    line = ser.readline()
    if not line:
        return None
    try:
        s = line.decode("utf-8", errors="ignore")
    except Exception:
        return None
    if SESSION_START not in s:
        return None
    buf_lines.append(s)
    start_time = time.time()
    while True:
        if SESSION_END in s:
            break
        if time.time() - start_time > 0.5:
            break
        line = ser.readline()
        if not line:
            break
        s = line.decode("utf-8", errors="ignore")
        buf_lines.append(s)
    return "".join(buf_lines)

# =========================================================
# 位置推定（Gauss-Newton with Dynamic Z-Constraint）
# =========================================================
Z_CONSTRAINT_WEIGHT = 2.0 

def solve_position_gn(
    anchors: Dict[str, Tuple[float, float, float]],
    ranges_m: Dict[str, float],
    x0: Optional[np.ndarray] = None,
    target_z_m: float = 1.0
) -> Tuple[Optional[np.ndarray], float]:
    used = []
    for mac, rr in ranges_m.items():
        if mac in anchors:
            used.append((mac, anchors[mac], rr))
    if len(used) < 3:
        return None, float("inf")
    A = np.array([a for _, a, _ in used], dtype=np.float64)
    r = np.array([rr for _, _, rr in used], dtype=np.float64)
    if x0 is None:
        x = np.mean(A, axis=0).copy()
        x[2] = target_z_m
    else:
        x = x0.astype(np.float64).copy()
    for _ in range(MAX_ITERS):
        diff = x[None, :] - A
        d = np.linalg.norm(diff, axis=1) + 1e-12
        f = d - r
        J = diff / d[:, None]
        z_err = x[2] - target_z_m
        w_sqrt = np.sqrt(Z_CONSTRAINT_WEIGHT)
        f_aug = np.append(f, w_sqrt * z_err)
        row_z = np.array([[0, 0, w_sqrt]])
        J_aug = np.vstack([J, row_z])
        H = J_aug.T @ J_aug
        g = J_aug.T @ f_aug
        H = H + 1e-4 * np.eye(3)
        try:
            dx = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            return None, float("inf")
        x_new = x + dx
        if np.linalg.norm(dx) < TOL:
            x = x_new
            break
        x = x_new
    diff = x[None, :] - A
    d = np.linalg.norm(diff, axis=1) + 1e-12
    f_final = d - r
    rms = float(np.sqrt(np.mean(f_final * f_final)))
    return x, rms

# =========================================================
# UDP送信（GUI用）
# =========================================================
class UdpSender:
    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def send_dict(self, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.sock.sendto(data, self.addr)

# =========================================================
# NMEA 生成（Tokyo時刻）
# =========================================================
TOKYO_TZ = ZoneInfo("Asia/Tokyo")

def nmea_checksum(body: str) -> str:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f"{cs:02X}"

def deg_to_nmea_lat(lat_deg: float) -> Tuple[str, str]:
    hemi = "N" if lat_deg >= 0 else "S"
    d = abs(lat_deg)
    deg_i = int(d)
    min_f = (d - deg_i) * 60.0
    return f"{deg_i:02d}{min_f:07.4f}", hemi

def deg_to_nmea_lon(lon_deg: float) -> Tuple[str, str]:
    hemi = "E" if lon_deg >= 0 else "W"
    d = abs(lon_deg)
    deg_i = int(d)
    min_f = (d - deg_i) * 60.0
    return f"{deg_i:03d}{min_f:07.4f}", hemi

def now_tokyo() -> datetime:
    return datetime.now(TOKYO_TZ)

def build_gga(now_tokyo_dt, lat_deg, lon_deg, alt_m, fix_quality=1, num_sats=8, hdop=1.0, geoid_sep_m=0.0):
    hhmmss = now_tokyo_dt.strftime("%H%M%S")
    lat_n, lat_hemi = deg_to_nmea_lat(lat_deg)
    lon_n, lon_hemi = deg_to_nmea_lon(lon_deg)
    body = (f"GPGGA,{hhmmss},{lat_n},{lat_hemi},{lon_n},{lon_hemi},{fix_quality:d},"
            f"{num_sats:02d},{hdop:.1f},{alt_m:.2f},M,{geoid_sep_m:.2f},M,,")
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"

def build_rmc(now_tokyo_dt, lat_deg, lon_deg, status="A", sog_knots=0.0, cog_deg=0.0):
    hhmmss = now_tokyo_dt.strftime("%H%M%S")
    ddmmyy = now_tokyo_dt.strftime("%d%m%y")
    lat_n, lat_hemi = deg_to_nmea_lat(lat_deg)
    lon_n, lon_hemi = deg_to_nmea_lon(lon_deg)
    body = (f"GPRMC,{hhmmss},{status},{lat_n},{lat_hemi},{lon_n},{lon_hemi},"
            f"{sog_knots:.1f},{cog_deg:.1f},{ddmmyy},,,A")
    cs = nmea_checksum(body)
    return f"${body}*{cs}\r\n"

# =========================================================
# ArduPilot送信（NMEA）
# =========================================================
class ArduPilotUartSender:
    def __init__(self, ser_out: serial.Serial):
        self.ser_out = ser_out
    def send_position_nmea(self, lat_deg: float, lon_deg: float, alt_m: float, rms_m: float) -> None:
        t_tokyo = now_tokyo()
        if (not np.isfinite(lat_deg)) or (not np.isfinite(lon_deg)) or (not np.isfinite(alt_m)):
            return
        status = "A"
        calc_hdop = max(0.6, rms_m * 5.0)
        if calc_hdop > 99.9: calc_hdop = 99.9
        
        gga = build_gga(t_tokyo, float(lat_deg), float(lon_deg), float(alt_m), 
                        fix_quality=1, num_sats=12, hdop=calc_hdop)
        rmc = build_rmc(t_tokyo, float(lat_deg), float(lon_deg), status=status)
        self.ser_out.write(gga.encode("ascii"))
        self.ser_out.write(rmc.encode("ascii"))
        self.ser_out.flush()

# =========================================================
# 平滑化
# =========================================================
def update_deque_and_prune(buf: deque, t_now: float, x: float, y: float, z: float, window_sec: float) -> None:
    buf.append((t_now, x, y, z))
    t_min = t_now - window_sec
    while buf and buf[0][0] < t_min:
        buf.popleft()

def mean_xyz(buf: deque) -> Optional[Tuple[float, float, float]]:
    if not buf:
        return None
    xs = [v[1] for v in buf]
    ys = [v[2] for v in buf]
    zs = [v[3] for v in buf]
    return (float(sum(xs) / len(xs)), float(sum(ys) / len(ys)), float(sum(zs) / len(zs)))

# =========================================================
# 追加：Smooth座標を使ったRMSE再計算関数
# =========================================================
def calc_rms_from_coord(
    anchors: Dict[str, Tuple[float, float, float]],
    ranges_m: Dict[str, float],
    pos: Tuple[float, float, float]
) -> float:
    if not ranges_m:
        return 0.0
    sq_errors = []
    px, py, pz = pos
    for mac, r_meas in ranges_m.items():
        if mac in anchors:
            ax, ay, az = anchors[mac]
            dist_calc = np.sqrt((px - ax)**2 + (py - ay)**2 + (pz - az)**2)
            sq_errors.append((dist_calc - r_meas) ** 2)
    if not sq_errors:
        return 0.0
    return np.sqrt(np.mean(sq_errors))

# =========================================================
# メイン
# =========================================================
def main():
    fcsv = open(LOG_PATH, "w", newline="")
    writer = csv.writer(fcsv)
    writer.writerow([
        "ts", "seq", "num_ranges",
        "x_m_raw", "y_m_raw", "z_m_raw", "rms_m_corrected", # ラベル名をわかりやすく変更
        "x_m_smooth", "y_m_smooth", "z_m_smooth",
        "lat_deg_smooth", "lon_deg_smooth", "alt_m_smooth",
        "sent_position"
    ])
    fcsv.flush()

    udp = UdpSender(UDP_HOST, UDP_PORT)
    ser_in = serial.Serial(SERIAL_PORT_IN, BAUDRATE_IN, timeout=TIMEOUT_IN)
    ser_out = serial.Serial(SERIAL_PORT_OUT, BAUDRATE_OUT, timeout=TIMEOUT_OUT)
    ap = ArduPilotUartSender(ser_out)

    try:
        ser_in.write(INIT_COMMAND.encode("utf-8"))
        ser_in.flush()
        time.sleep(AFTER_INIT_SLEEP_SEC)
    except Exception:
        pass

    last_pos = None
    last_send_mono = 0.0
    last_smooth_update_mono = 0.0
    pos_buf = deque()
    smooth_xyz = None
    smooth_latlonalt = None
    
    current_rms = 0.0
    last_calc_time = 0.0
    last_calc_pos = None
    last_valid_z = 0.1

    print("=== UWB -> GN -> (0.1s mean) -> LatLon -> ArduPilot (NMEA) @20Hz ===")
    print(f"IN : {SERIAL_PORT_IN} @ {BAUDRATE_IN}")
    print(f"OUT: {SERIAL_PORT_OUT} @ {BAUDRATE_OUT}")
    print(f"ORIGIN lat/lon/alt: {ORIGIN_LAT_DEG}, {ORIGIN_LON_DEG}, {ORIGIN_ALT_M}")
    print(f"SEND: {SEND_HZ:.1f} Hz, smooth window={SMOOTH_WINDOW_SEC:.3f}s, smooth update={SMOOTH_UPDATE_SEC:.3f}s")
    print("--------------------------------------------------")

    try:
        while True:
            block = read_session_block(ser_in)
            if block is not None:
                sess = parse_session(block)
                seq = sess.get("sequence_number", None)
                ranges_m = sess.get("ranges_m", {})

                usable = sum(1 for mac in ANCHOR_ORDER if mac in ranges_m)
                if usable >= 3:
                    x0 = last_pos if last_pos is not None else None
                    pos, rms = solve_position_gn(ANCHORS, ranges_m, x0=x0, target_z_m=last_valid_z)

                    if pos is not None:
                        # --- 1. 速度計算 (m/s) -----------------------------
                        now = time.time()
                        speed_mps = 0.0
                        if last_calc_pos is not None:
                            dt = now - last_calc_time
                            if dt > 0.001:
                                dist_2d = np.linalg.norm(pos[:2] - last_calc_pos[:2])
                                speed_mps = dist_2d / dt
                        
                        last_calc_time = now
                        last_calc_pos = pos
                        last_pos = pos

                        x_m_raw, y_m_raw, z_m_raw = float(pos[0]), float(pos[1]), float(pos[2])

                        # --- Z値の更新 ---
                        alpha = 0.1
                        last_valid_z = last_valid_z * (1.0 - alpha) + z_m_raw * alpha

                        t_mono = time.monotonic()
                        update_deque_and_prune(pos_buf, t_mono, x_m_raw, y_m_raw, z_m_raw, SMOOTH_WINDOW_SEC)

                        if (t_mono - last_smooth_update_mono) >= SMOOTH_UPDATE_SEC:
                            m = mean_xyz(pos_buf)
                            if m is not None:
                                smooth_xyz = m
                                lat, lon, alt = enu_to_latlon_alt(smooth_xyz[0], smooth_xyz[1], smooth_xyz[2])
                                smooth_latlonalt = (lat, lon, alt)

                                # --- 2. Smooth座標ベースでRMSEを再計算 ---
                                recalc_rms = calc_rms_from_coord(ANCHORS, ranges_m, smooth_xyz)

                                # --- 3. 速度補正 ---
                                SPEED_PENALTY_COEFF = 0.05 
                                discount = speed_mps * SPEED_PENALTY_COEFF
                                current_rms = max(0.01, recalc_rms - discount)

                            last_smooth_update_mono = t_mono

                        if smooth_xyz is not None and smooth_latlonalt is not None:
                            # ログ保存 (RMSは補正後のものを使用)
                            writer.writerow([
                                time.time(), seq, len(ranges_m),
                                x_m_raw, y_m_raw, z_m_raw, current_rms,
                                smooth_xyz[0], smooth_xyz[1], smooth_xyz[2],
                                smooth_latlonalt[0], smooth_latlonalt[1], smooth_latlonalt[2],
                                ""
                            ])
                            fcsv.flush()

                            try:
                                udp.send_dict({
                                    "ok": True, "type": "uwb_solution", "seq": seq,
                                    "x_m": smooth_xyz[0], "y_m": smooth_xyz[1], "z_m": smooth_xyz[2],
                                    "xyz_mm": [smooth_xyz[0]*1000.0, smooth_xyz[1]*1000.0, smooth_xyz[2]*1000.0],
                                    "rms_m": current_rms,  # GUIへも補正後RMSを送る
                                    "lat_deg": smooth_latlonalt[0], "lon_deg": smooth_latlonalt[1], "alt_m": smooth_latlonalt[2],
                                    "ts": time.time(),
                                })
                            except Exception:
                                pass

            # --- 送信は 20Hz ---
            now_mono = time.monotonic()
            if (now_mono - last_send_mono) >= SEND_INTERVAL_SEC:
                if smooth_latlonalt is not None:
                    try:
                        ap.send_position_nmea(
                            lat_deg=smooth_latlonalt[0],
                            lon_deg=smooth_latlonalt[1],
                            alt_m=smooth_latlonalt[2],
                            rms_m=current_rms # ArduPilotへも補正後RMSを送る
                        )
                    except Exception:
                        pass
                last_send_mono = now_mono

    except KeyboardInterrupt:
        print("\n[Ctrl+C] stopping...")

    finally:
        try: ser_in.write(STOP_COMMAND.encode("utf-8")); ser_in.flush()
        except: pass
        try: ser_in.close()
        except: pass
        try: ser_out.close()
        except: pass
        try: fcsv.close()
        except: pass

if __name__ == "__main__":
    main()