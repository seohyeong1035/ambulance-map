#!/usr/bin/env python3
# Read AMB CSV lines from ESP32 receiver and write state.json for the map.

import sys, os, json, time
import serial

from math import radians, sin, cos, sqrt, atan2

RCV_LAT = 35.8879
RCV_LON = 128.6118

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000
    p1, p2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlmb = radians(lon2 - lon1)
    a = sin(dphi/2)**2 + cos(p1)*cos(p2)*sin(dlmb/2)**2
    return 2 * R * atan2(sqrt(a), sqrt(1-a))

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 115200
DIR  = os.path.dirname(os.path.abspath(__file__))
STATE = os.path.join(DIR, "state.json")

# default state (초기값)
state = {
    "lat": 35.88718,
    "lon": 128.61202,
    "speed": 0.0,
    "heading": 0.0,
    "rssi": -99,
    "emergency": False,
    "approaching": False
}

def write_state(d: dict):
    tmp = STATE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, STATE)

def main():
    print(f"[RX] open {PORT} {BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1)

    ema = None          # RSSI 지수이동평균
    prev_ema = None
    ALPHA = 0.3         # EMA 가중치

    while True:
        line = ser.readline().decode("utf-8", "ignore").strip()
        if not line:
            continue
        # 기대 포맷: AMB,seq,lat,lon,speed,heading,rssi,fix,emergency,mac
        if not line.startswith("AMB,"):
            # 필요하면 디버그 출력
            # print(line)
            continue

        parts = line.split(",")
        if len(parts) < 10:
            continue

        try:
            lat   = float(parts[2])
            lon   = float(parts[3])
            if lat > 90:
                lat = lat / 1e6
            if lon > 180:
                lon = lon / 1e6
            spd   = float(parts[4])
            hdg   = float(parts[5])
            rssi  = int(parts[6])
            fix   = int(parts[7]) != 0
            emer  = int(parts[8]) != 0
        except Exception:
            continue

        # RSSI EMA 계산
        prev_ema = ema
        ema = rssi if ema is None else int(ALPHA * rssi + (1 - ALPHA) * ema)
        rising = (prev_ema is not None and (ema - prev_ema) >= 2)

        state["lat"] = lat if fix else state["lat"]
        state["lon"] = lon if fix else state["lon"]
        state["speed"] = spd
        state["heading"] = hdg
        state["rssi"] = rssi
        state["emergency"] = bool(emer)
    # --- 거리 기반 접근 판정 (5 m) ---
        if fix:
            dist = haversine_m(lat, lon, RCV_LAT, RCV_LON)
            state["dist_m"] = round(dist, 1)
        else:
            dist = None
            state["dist_m"] = None

    # 5 m 이내면 배너 ON
        state["approaching"] = (dist is not None and dist <= 5.0)


        write_state(state)
        print(f"[AMB] fix={int(fix)} emer={int(emer)} rssi={rssi} "
              f"lat={state['lat']:.6f} lon={state['lon']:.6f}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")


