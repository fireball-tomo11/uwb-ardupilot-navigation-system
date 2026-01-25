#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import socket
import threading
from typing import Optional, Dict, Any, Tuple

from flask import Flask, Response, render_template_string


# =========================================================
# 設定
# =========================================================

UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 9900

# アンカー座標（m）mac_addressと対応させる
ANCHORS: Dict[str, Tuple[float, float, float]] = {
    "0x0001": (0.0, 0.0, 0.5),
    "0x0002": (2.0, 0.0, 2.0),
    "0x0003": (0.0, 2.0, 2.0),
    "0x0004": (2.0, 2.0, 0.5),
}

PAD_MM = 200.0   # 表示範囲の余白(mm)
HIST_MAX = 800   # 軌跡の保持数


# =========================================================
# 共有状態
# =========================================================

latest_lock = threading.Lock()
latest_payload: Optional[Dict[str, Any]] = None
stop_event = threading.Event()


def anchors_bounds_mm() -> Tuple[float, float, float, float, float, float]:
    xs = [v[0] * 1000.0 for v in ANCHORS.values()]
    ys = [v[1] * 1000.0 for v in ANCHORS.values()]
    zs = [v[2] * 1000.0 for v in ANCHORS.values()]
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


AX_MINX, AX_MAXX, AX_MINY, AX_MAXY, AX_MINZ, AX_MAXZ = anchors_bounds_mm()
AX_MINX -= PAD_MM; AX_MAXX += PAD_MM
AX_MINY -= PAD_MM; AX_MAXY += PAD_MM
AX_MINZ -= PAD_MM; AX_MAXZ += PAD_MM


# =========================================================
# UDP受信スレッド
# =========================================================

def udp_worker():
    global latest_payload
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
    sock.settimeout(0.5)

    while not stop_event.is_set():
        try:
            data, _addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except Exception as e:
            with latest_lock:
                latest_payload = {"ts": time.time(), "ok": False, "error": f"udp_error: {repr(e)}"}
            continue

        try:
            p = json.loads(data.decode("utf-8", errors="ignore"))
            p["recv_ts"] = time.time()
        except Exception as e:
            with latest_lock:
                latest_payload = {"ts": time.time(), "ok": False, "error": f"json_error: {repr(e)}"}
            continue

        with latest_lock:
            latest_payload = p

    sock.close()


# =========================================================
# Flask
# =========================================================

app = Flask(__name__)

HTML = r"""
<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <title>UWB 3D Plotly GUI</title>
  <style>
    body { font-family: sans-serif; padding: 16px; }
    .muted { color:#555; font-size: 12px; }
    .row { display:flex; gap:16px; flex-wrap: wrap; align-items:flex-start; }
    .card { border:1px solid #ccc; border-radius:12px; padding:12px 16px; }
    .big { font-size: 22px; font-weight: 700; }
    .ok { color:#0a7; font-weight:700; }
    .ng { color:#c33; font-weight:700; }
    .kpi { font-size: 14px; line-height: 1.6; }
    .kpi b { display:inline-block; width: 130px; }
    pre { white-space: pre-wrap; word-break: break-word; font-size: 12px; margin: 0; }
  </style>

  <!-- Plotly CDN -->
  <script src="https://cdn.plot.ly/plotly-2.30.0.min.js"></script>
</head>
<body>
  <h2>UWB 3D Plotly GUI</h2>
  <div class="muted">
    UDP listen: {{udp_ip}}:{{udp_port}} /
    Axis(mm): X[{{minx}},{{maxx}}] Y[{{miny}},{{maxy}}] Z[{{minz}},{{maxz}}]
  </div>

  <div class="row" style="margin-top:12px;">
    <div class="card" style="min-width:360px;">
      <div id="status" class="big">---</div>

      <div style="margin-top:10px;">
        <div class="big" id="xyz">X=---, Y=---, Z=---</div>
        <div class="muted" id="llh">lat=---, lon=---, alt=---</div>
        <div class="muted" id="meta">seq=--- / ts=---</div>
      </div>

      <div style="margin-top:10px;" class="kpi">
        <div><b>RMSE</b><span id="rmse">---</span></div>
        <div class="muted" style="margin-top:6px;">
          RMSEは距離整合性の指標（小さいほど良い）
        </div>
      </div>

      <div style="margin-top:10px;">
        <b>Raw payload</b>
        <pre id="raw">---</pre>
      </div>
    </div>

    <div class="card">
      <div><b>3D (drag rotate / zoom) : Anchors + Trajectory + Current</b></div>
      <div id="plot3d" style="width: 720px; height: 560px;"></div>
      <div class="muted" style="margin-top:6px;">
        マウスで回転 / ホイールでズーム / ダブルクリックでリセット
      </div>
    </div>

    <div class="card">
      <div><b>X (mm)</b></div>
      <div id="plotx" style="width: 720px; height: 200px;"></div>
      <div><b>Y (mm)</b></div>
      <div id="ploty" style="width: 720px; height: 200px;"></div>
      <div><b>Z (mm)</b></div>
      <div id="plotz" style="width: 720px; height: 200px;"></div>
      <div class="muted" style="margin-top:6px;">
        軸範囲はアンカー座標固定（+余白）
      </div>
    </div>
  </div>

<script>
  // -----------------------------
  // 固定軸範囲（mm）
  // -----------------------------
  const AX = {
    minx: {{minx}}, maxx: {{maxx}},
    miny: {{miny}}, maxy: {{maxy}},
    minz: {{minz}}, maxz: {{maxz}}
  };

  const ANCHORS = {{ anchors | tojson }};
  const HIST_MAX = {{hist_max}};

  // -----------------------------
  // DOM
  // -----------------------------
  const statusEl = document.getElementById("status");
  const xyzEl = document.getElementById("xyz");
  const metaEl = document.getElementById("meta");
  const rawEl = document.getElementById("raw");
  const rmseEl = document.getElementById("rmse");
  const llhEl = document.getElementById("llh");

  // -----------------------------
  // 履歴（mm）
  // -----------------------------
  let histX = [];
  let histY = [];
  let histZ = [];
  let histT = []; // サンプル番号(0..)

  function pushHist(x,y,z){
    const n = histT.length ? (histT[histT.length-1] + 1) : 0;
    histT.push(n);
    histX.push(x); histY.push(y); histZ.push(z);

    if(histT.length > HIST_MAX){
      histT.shift();
      histX.shift(); histY.shift(); histZ.shift();
    }
  }

  // -----------------------------
  // 3D初期化
  // -----------------------------
  function init3D(){
    const ax = [];
    const ay = [];
    const az = [];
    const labels = [];
    for(const k in ANCHORS){
      ax.push(ANCHORS[k][0]*1000.0);
      ay.push(ANCHORS[k][1]*1000.0);
      az.push(ANCHORS[k][2]*1000.0);
      labels.push(k);
    }

    const anchorsTrace = {
      type: "scatter3d",
      mode: "markers+text",
      x: ax, y: ay, z: az,
      text: labels,
      textposition: "top center",
      marker: { size: 5 }
    };

    const trajTrace = {
      type: "scatter3d",
      mode: "lines",
      x: [], y: [], z: [],
      line: { width: 3 }
    };

    const curTrace = {
      type: "scatter3d",
      mode: "markers",
      x: [], y: [], z: [],
      marker: { size: 6 }
    };

    const layout = {
      margin: {l:0, r:0, t:10, b:0},
      scene: {
        xaxis: {title:"X (mm)", range:[AX.minx, AX.maxx], showgrid:true, zeroline:true},
        yaxis: {title:"Y (mm)", range:[AX.miny, AX.maxy], showgrid:true, zeroline:true},
        zaxis: {title:"Z (mm)", range:[AX.minz, AX.maxz], showgrid:true, zeroline:true},
        aspectmode: "data"
      },
      showlegend: false
    };

    Plotly.newPlot("plot3d", [anchorsTrace, trajTrace, curTrace], layout, {displayModeBar:true});
  }

  // -----------------------------
  // 時系列初期化（3段）
  // -----------------------------
  function initTS(divId, title, ymin, ymax){
    const trace = {
      type: "scatter",
      mode: "lines",
      x: [], y: []
    };
    const layout = {
      margin: {l:50, r:15, t:18, b:35},
      xaxis: {title:"sample", showgrid:true},
      yaxis: {title:title, range:[ymin, ymax], showgrid:true, zeroline:true},
      showlegend: false
    };
    Plotly.newPlot(divId, [trace], layout, {displayModeBar:false});
  }

  // -----------------------------
  // 更新描画（3D + TS）
  // -----------------------------
  function updatePlots(x,y,z){
    Plotly.restyle("plot3d", {x:[histX], y:[histY], z:[histZ]}, [1]);
    Plotly.restyle("plot3d", {x:[[x]],  y:[[y]],  z:[[z]]},  [2]);

    Plotly.restyle("plotx", {x:[histT], y:[histX]}, [0]);
    Plotly.restyle("ploty", {x:[histT], y:[histY]}, [0]);
    Plotly.restyle("plotz", {x:[histT], y:[histZ]}, [0]);
  }

  // -----------------------------
  // RMSE 表示
  // -----------------------------
  function updateErrorKPI(p){
    const rmse_m = p.rms_m;

    if(typeof rmse_m === "number"){
      rmseEl.textContent = ` ${ (rmse_m*1000.0).toFixed(1) } mm`;
    } else {
      rmseEl.textContent = " ---";
    }
  }

  // -----------------------------
  // 初期化実行
  // -----------------------------
  init3D();
  initTS("plotx", "X (mm)", AX.minx, AX.maxx);
  initTS("ploty", "Y (mm)", AX.miny, AX.maxy);
  initTS("plotz", "Z (mm)", AX.minz, AX.maxz);

  // -----------------------------
  // SSE
  // -----------------------------
  const es = new EventSource("/stream");
  es.onmessage = (ev) => {
    const p = JSON.parse(ev.data);

    const ok = !!p.ok;
    statusEl.textContent = ok ? "OK" : "NO FIX";
    statusEl.className = ok ? "big ok" : "big ng";

    const xyz = p.xyz_mm;
    if (xyz) {
      const x = xyz[0], y = xyz[1], z = xyz[2];
      xyzEl.textContent = `X=${x.toFixed(1)}  Y=${y.toFixed(1)}  Z=${z.toFixed(1)}  (mm)`;

      pushHist(x,y,z);
      updatePlots(x,y,z);
    } else {
      xyzEl.textContent = "X=---, Y=---, Z=---";
    }

    // lat/lon/alt 表示（送信側: lat_deg, lon_deg, alt_m）
    const lat = p.lat_deg;
    const lon = p.lon_deg;
    const alt = p.alt_m;
    if (typeof lat === "number" && typeof lon === "number" && typeof alt === "number") {
      llhEl.textContent = `lat=${lat.toFixed(7)}  lon=${lon.toFixed(7)}  alt=${alt.toFixed(2)} (m)`;
    } else {
      llhEl.textContent = "lat=---, lon=---, alt=---";
    }

    updateErrorKPI(p);

    const showTs = (typeof p.recv_ts === "number") ? p.recv_ts : (p.ts ?? 0);
    metaEl.textContent = `seq=${p.seq ?? p.sequence_number ?? "---"} / ts=${showTs.toFixed(3)}`;

    rawEl.textContent = JSON.stringify(p, null, 2);
  };
</script>
</body>
</html>
"""


@app.get("/")
def index():
    return render_template_string(
        HTML,
        udp_ip=UDP_LISTEN_IP,
        udp_port=UDP_LISTEN_PORT,
        minx=AX_MINX, maxx=AX_MAXX,
        miny=AX_MINY, maxy=AX_MAXY,
        minz=AX_MINZ, maxz=AX_MAXZ,
        anchors=ANCHORS,
        hist_max=HIST_MAX,
    )


@app.get("/stream")
def stream():
    def gen():
        last_ts = None
        while True:
            time.sleep(0.03)
            with latest_lock:
                cur = latest_payload
            if cur is None:
                continue
            ts = cur.get("recv_ts", cur.get("ts"))
            if ts == last_ts:
                continue
            last_ts = ts
            yield f"data: {json.dumps(cur, ensure_ascii=False)}\n\n"

    return Response(gen(), mimetype="text/event-stream")


def main():
    th = threading.Thread(target=udp_worker, daemon=True)
    th.start()

    print("[INFO] Plotly GUI server starting...")
    print(f"[INFO] UDP listen: {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")
    print("[INFO] Open: http://127.0.0.1:8000")

    try:
        app.run(host="0.0.0.0", port=8000, debug=False, threaded=True)
    finally:
        stop_event.set()


if __name__ == "__main__":
    main()
