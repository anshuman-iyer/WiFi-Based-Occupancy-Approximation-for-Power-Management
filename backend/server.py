"""
===============================================================
ESP32 OCCUPANCY INTELLIGENCE SERVER  (v4 -- 3-node split hardware)
===============================================================

v3 drops the v2 forecasting/calibration machinery entirely, per a
scope change: no future prediction (no labeled dataset is coming),
just the cleanest possible *current* occupancy number.

Two rule-based (non-ML) noise filters replace Holt/EWMA smoothing:

  1. RSSI proximity gate -- a device whose strongest observed
     signal is weaker than RSSI_PRESENCE_THRESHOLD is treated as
     likely bleed-through from an adjacent room/hallway (hostel
     WiFi is everywhere), not someone actually in the room, and is
     excluded before counting. Tune this by testing empirically --
     stand just outside the door and watch where the count drops.

  2. Confirm/candidate hysteresis -- a device must be seen in
     CONFIRM_AFTER_POLLS consecutive polls before it counts toward
     occupancy (suppresses one-off false positives), and must be
     missing for DROP_AFTER_MISSED_POLLS consecutive polls before
     it's dropped (suppresses flicker from a single transient poll
     failure -- the ESP32 nodes already handle real device timeout
     on their own 30s clock; this is a second, independent layer
     against fusion-level jitter).

The reported room_count is the count of currently CONFIRMED devices
-- not the raw fused count, not a smoothed/extrapolated estimate.
===============================================================

Run:
    pip install flask requests
    python3 server.py

Open:
    http://localhost:8000

Architecture:

ESP32 nodes
     |
/api/devices
     |
multi-node fusion (shared salt across nodes -> same device, same hash)
     |
RSSI proximity gate (drop weak/bleed-through signals)
     |
confirm/candidate hysteresis (debounce over consecutive polls)
     |
confidence / unusual-jump flag (both describe the CURRENT state)
     |
SQLite persistence (diagnostic history, no ground-truth labels)
     |
dashboard

This intentionally does not attempt to predict future occupancy or
learn a device-count -> person-count mapping: both require labeled
ground-truth data that isn't coming. Everything here is rule-based
and tunable by hand -- RSSI_PRESENCE_THRESHOLD, CONFIRM_AFTER_POLLS,
and DROP_AFTER_MISSED_POLLS are the three knobs, all near the top
of this file.
===============================================================
"""

import time
import json
import sqlite3
import threading
import statistics
from collections import deque

import requests
from flask import Flask, jsonify, render_template_string, request, Response


# =============================================================
# CONFIGURATION
# =============================================================

app = Flask(__name__)

NODES = {
    "NODE-01": "http://192.168.137.203",  # confirmed from this board's actual Serial boot log
    "NODE-02": "http://192.168.137.202",  # UNVERIFIED -- check this board's own Serial output
    "NODE-03": "http://192.168.137.201",  # UNVERIFIED -- check this board's own Serial output
}

# NODE-01 has the MIC, NODE-02 has the BUZZER, NODE-03 is display-only.
# This board's clap count is read from its own /api/status (which is
# already polled every 2s for other purposes) rather than having the
# ESP32 push to the server, which would require it to know the
# server's IP -- an extra fragile dependency worth avoiding given how
# much trouble the network IPs have already caused today.
CLAP_NODE = "NODE-01"       # mic board

# Display mode cycle, advanced by a double-clap on CLAP_NODE.
MODE_COMBINED = "combined"  # all 3 strips show the same fused room count
MODE_AREA     = "area"      # each strip shows its OWN node's local count;
                             # brightness scales relative to the busiest node
MODE_STANDBY  = "standby"   # all 3 strips are just a steady light, ignoring occupancy

MODE_CYCLE = [MODE_COMBINED, MODE_AREA, MODE_STANDBY]

POLL_INTERVAL_S = 2
STALE_AFTER_MS = 30000

# Number of in-memory samples kept for the live chart (SQLite holds
# the full history regardless -- this is just what the dashboard
# chart pulls from without re-querying the DB every second).
HISTORY_SIZE = 1800

# ---- Noise filters (rule-based, no training data needed) ----

# A device whose strongest observed RSSI is weaker than this is
# treated as likely bleed-through from outside the room, not an
# actual occupant. Tune empirically: walk to the doorway and watch
# where the confirmed count drops.
RSSI_PRESENCE_THRESHOLD = -80  # dBm

# A device must appear in this many consecutive polls before it
# counts toward occupancy (suppresses one-off false positives).
CONFIRM_AFTER_POLLS = 2

# A device must be missing for this many consecutive polls before
# it's dropped from the count (suppresses flicker from a single
# transient poll failure -- independent of the ESP32's own 30s
# device timeout, which handles real physical departure).
DROP_AFTER_MISSED_POLLS = 2

NODE_TIMEOUT_S = 8

DB_PATH = "occupancy_history_v3.db"  # separate file from any v2 DB (different schema)


# =============================================================
# STATE
# =============================================================

lock = threading.Lock()
db_lock = threading.Lock()

latest = {
    "room_count": 0,
    "raw_count": 0,
    "candidate_count": 0,
    "confidence": 0,

    "mean_rssi": 0,
    "rssi_stddev": 0,

    "node_agreement": 0,
    "probe_rate": 0,

    "unusual_jump": False,
    "jump_score": 0,

    "display_mode": MODE_COMBINED,

    "per_node_status": {},
    "fused_devices": [],
    "rssi_threshold": RSSI_PRESENCE_THRESHOLD,

    "last_update": None,
}

history = deque(maxlen=HISTORY_SIZE)

# Per-device hysteresis state, keyed by hashed device id:
#   {"seen_streak": int, "missed_streak": int, "confirmed": bool,
#    "last_rssi": int, "seen_by": [node names]}
device_states = {}

# Display mode state, driven by claps detected on CLAP_NODE.
current_display_mode = MODE_COMBINED
last_seen_clap_count = None  # None until the first successful read establishes a baseline


# =============================================================
# DATABASE
# =============================================================

def init_db():
    with db_lock:
        conn = sqlite3.connect(DB_PATH)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ts REAL NOT NULL,
                raw_count INTEGER,
                confirmed_count INTEGER,
                candidate_count INTEGER,
                node_counts TEXT,
                mean_rssi REAL,
                rssi_stddev REAL,
                probe_rate INTEGER,
                confidence REAL,
                node_agreement REAL
            )
        """)
        conn.commit()
        conn.close()


def persist_row(
    ts, raw_count, confirmed_count, candidate_count,
    node_counts, mean_rssi, rssi_stddev,
    probe_rate, confidence, node_agreement
):
    with db_lock:
        conn = sqlite3.connect(DB_PATH)
        conn.execute(
            """
            INSERT INTO history (
                ts, raw_count, confirmed_count, candidate_count,
                node_counts, mean_rssi, rssi_stddev,
                probe_rate, confidence, node_agreement
            ) VALUES (?,?,?,?,?,?,?,?,?,?)
            """,
            (
                ts, raw_count, confirmed_count, candidate_count,
                json.dumps(node_counts), mean_rssi, rssi_stddev,
                probe_rate, confidence, node_agreement
            )
        )
        conn.commit()
        conn.close()


# =============================================================
# UTILITY FUNCTIONS
# =============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


# =============================================================
# FUSION
# =============================================================

def fuse_devices(node_data):
    """
    Merge devices detected by multiple nodes.

    Device IDs are already hashed by the ESP32 (with a shared fixed
    salt across all nodes, so the same physical device produces the
    same hash on every node -- this is what makes fusion possible).

    Records: id, strongest RSSI, which nodes saw it, node count.
    """

    merged = {}

    for node_name, data in node_data.items():
        for dev in data.get("devices", []):
            try:
                device_id = int(dev["id"])
                rssi = int(dev["rssi"])
            except (KeyError, TypeError, ValueError):
                continue

            if device_id not in merged:
                merged[device_id] = {"id": device_id, "rssi": rssi, "seen_by": []}

            merged[device_id]["rssi"] = max(merged[device_id]["rssi"], rssi)

            if node_name not in merged[device_id]["seen_by"]:
                merged[device_id]["seen_by"].append(node_name)

    return list(merged.values())


# =============================================================
# CONFIDENCE
# =============================================================

def calculate_confidence(active_nodes, total_nodes, fused_devices, node_counts, rssi_values):
    """
    Sensor-quality score, NOT a probability that the count is correct.
    Based on: node availability, node agreement, RSSI quality,
    multi-node observation ratio.
    """

    if total_nodes == 0:
        return 0

    availability = active_nodes / total_nodes

    if node_counts:
        mean_count = statistics.mean(node_counts)
        if mean_count > 0:
            deviation = statistics.mean(abs(x - mean_count) for x in node_counts)
            agreement = 1.0 - clamp(deviation / max(mean_count, 1), 0, 1)
        else:
            agreement = 1.0
    else:
        agreement = 0

    if rssi_values:
        avg_rssi = statistics.mean(rssi_values)
        rssi_quality = clamp((avg_rssi + 90) / 50, 0, 1)  # -90dBm weak, -40dBm strong
    else:
        rssi_quality = 0

    multi_node_ratio = 0
    if fused_devices:
        multi_node_ratio = sum(len(d["seen_by"]) > 1 for d in fused_devices) / len(fused_devices)

    confidence = (
        availability * 0.35 +
        agreement * 0.30 +
        rssi_quality * 0.15 +
        multi_node_ratio * 0.20
    )

    return round(confidence * 100, 1)


# =============================================================
# PRESENCE CONFIRMATION (RSSI gate + confirm/candidate hysteresis)
# =============================================================
#
# Runs once per poll on the current fused device list. Rule-based,
# no training data required:
#
#   1. Drop any device whose strongest RSSI is weaker than
#      RSSI_PRESENCE_THRESHOLD (likely bleed-through from outside
#      the room).
#   2. A device seen this poll gets its seen_streak incremented and
#      missed_streak reset. Once seen_streak reaches
#      CONFIRM_AFTER_POLLS, it becomes "confirmed".
#   3. A device NOT seen this poll gets its missed_streak
#      incremented and seen_streak reset. Once missed_streak reaches
#      DROP_AFTER_MISSED_POLLS, it's removed entirely (whether or
#      not it was confirmed) -- this is the grace period that
#      absorbs a single transient poll failure without the count
#      flickering.
#
# Returns (confirmed_count, candidate_count).
# =============================================================

def update_presence(fused_devices):
    seen_ids = {}

    for dev in fused_devices:
        if dev["rssi"] < RSSI_PRESENCE_THRESHOLD:
            continue  # RSSI gate: treat as bleed-through, ignore entirely
        seen_ids[dev["id"]] = dev

    for device_id, dev in seen_ids.items():
        state = device_states.setdefault(device_id, {
            "seen_streak": 0, "missed_streak": 0, "confirmed": False,
        })
        state["seen_streak"] += 1
        state["missed_streak"] = 0
        state["last_rssi"] = dev["rssi"]
        state["seen_by"] = dev["seen_by"]

        if not state["confirmed"] and state["seen_streak"] >= CONFIRM_AFTER_POLLS:
            state["confirmed"] = True

    for device_id in list(device_states.keys()):
        if device_id not in seen_ids:
            state = device_states[device_id]
            state["seen_streak"] = 0
            state["missed_streak"] += 1

            if state["missed_streak"] >= DROP_AFTER_MISSED_POLLS:
                del device_states[device_id]

    confirmed_count = sum(1 for s in device_states.values() if s["confirmed"])
    candidate_count = sum(1 for s in device_states.values() if not s["confirmed"])

    return confirmed_count, candidate_count


# =============================================================
# UNUSUAL JUMP DETECTION (current-state, not predictive)
# =============================================================

def detect_unusual_jump(current):
    if len(history) < 20:
        return False, 0

    values = [x["occupancy"] for x in list(history)[-20:]]
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0
    if std < 1:
        std = 1

    z_score = abs(current - mean) / std
    return (z_score >= 3, round(z_score, 2))


# =============================================================
# POLLING
# =============================================================

def poll_once():
    merged_node_data = {}
    per_node_status = {}
    total_probes = 0

    for node_name, base_url in NODES.items():
        try:
            response = requests.get(f"{base_url}/api/devices", timeout=2)
            response.raise_for_status()
            data = response.json()
            now_ms = data.get("now_ms", 0)

            valid_devices = []
            for dev in data.get("devices", []):
                try:
                    age = now_ms - int(dev["last_seen_ms"])
                    if age <= STALE_AFTER_MS:
                        valid_devices.append(dev)
                except Exception:
                    continue

            node_count = len(valid_devices)
            merged_node_data[node_name] = {"devices": valid_devices}
            per_node_status[node_name] = {
                "ok": True, "devices": node_count, "url": base_url,
                "last_success": time.time(),
            }
            total_probes += node_count

        except Exception as exc:
            per_node_status[node_name] = {
                "ok": False, "devices": 0, "url": base_url,
                "error": str(exc), "last_success": None,
            }

    fused = fuse_devices(merged_node_data)
    raw_count = len(fused)

    node_counts = [s["devices"] for s in per_node_status.values() if s["ok"]]
    active_nodes = len(node_counts)
    total_nodes = len(NODES)

    rssi_values = [d["rssi"] for d in fused if isinstance(d.get("rssi"), int)]
    if rssi_values:
        mean_rssi = statistics.mean(rssi_values)
        rssi_stddev = statistics.stdev(rssi_values) if len(rssi_values) > 1 else 0
    else:
        mean_rssi = 0
        rssi_stddev = 0

    # ---- RSSI gate + confirm/candidate hysteresis (the actual denoising) ----
    confirmed_count, candidate_count = update_presence(fused)
    room_count = confirmed_count

    timestamp = time.time()

    # Per-node raw counts for the chart's per-node lines. None (not 0) for
    # a node that failed to respond this poll, so the chart shows a gap
    # instead of a misleading drop to zero.
    node_counts_by_name = {
        name: (s["devices"] if s["ok"] else None)
        for name, s in per_node_status.items()
    }

    history.append({
        "timestamp": timestamp,
        "occupancy": room_count,
        "raw_count": raw_count,
        "node_counts": node_counts,
        "node_counts_by_name": node_counts_by_name,
        "mean_rssi": mean_rssi,
        "probe_rate": total_probes,
    })

    confidence = calculate_confidence(active_nodes, total_nodes, fused, node_counts, rssi_values)

    if node_counts:
        mean_node_count = statistics.mean(node_counts)
        if mean_node_count:
            deviation = statistics.mean(abs(x - mean_node_count) for x in node_counts)
            agreement = clamp(1 - (deviation / max(mean_node_count, 1)), 0, 1)
        else:
            agreement = 1
    else:
        agreement = 0

    unusual_jump, jump_score = detect_unusual_jump(room_count)

    # ---- Check for a clap-triggered mode change ----
    # Reads the clap board's own /api/status (already fetched for every
    # other node's /api/devices call above -- this one extra request is
    # only to the single clap-hosting board).
    global current_display_mode, last_seen_clap_count

    clap_url = NODES.get(CLAP_NODE)
    if clap_url:
        try:
            status_resp = requests.get(f"{clap_url}/api/status", timeout=1)
            status_resp.raise_for_status()
            clap_count = status_resp.json().get("clap_count")

            if clap_count is not None:
                if last_seen_clap_count is None:
                    last_seen_clap_count = clap_count  # establish baseline, don't trigger on startup
                elif clap_count != last_seen_clap_count:
                    idx = MODE_CYCLE.index(current_display_mode)
                    current_display_mode = MODE_CYCLE[(idx + 1) % len(MODE_CYCLE)]
                    last_seen_clap_count = clap_count
                    print(f"[MODE] Clap detected on {CLAP_NODE} -> mode is now {current_display_mode}")
        except Exception as exc:
            print(f"[CLAP CHECK FAILED - {CLAP_NODE}]", exc)

    # ---- Push mode-appropriate display data to all 3 strips ----
    # COMBINED: identical payload (the fused count) to every node.
    # AREA: each node gets its OWN local count plus a brightness scale
    #       relative to the busiest node right now (computed here since
    #       only the server sees all three nodes at once).
    # STANDBY: payload content doesn't matter -- boards ignore it and
    #       just show a steady light.
    if current_display_mode == MODE_AREA:
        max_local_count = max(node_counts) if node_counts else 0

    for node_name, base_url in NODES.items():
        if current_display_mode == MODE_COMBINED:
            payload = {"mode": MODE_COMBINED, "count": room_count}
        elif current_display_mode == MODE_AREA:
            local_count = per_node_status.get(node_name, {}).get("devices", 0)
            scale = (local_count / max_local_count) if max_local_count > 0 else 0.0
            payload = {"mode": MODE_AREA, "count": local_count, "scale": round(scale, 2)}
        else:
            payload = {"mode": MODE_STANDBY}

        try:
            requests.post(f"{base_url}/api/actuator", json=payload, timeout=1)
        except Exception as exc:
            print(f"[DISPLAY PUSH FAILED - {node_name}]", exc)

    # A DB write failure must never block the stats/node-card update below --
    # that exact failure mode (exception here, before latest.update() runs)
    # is what caused boxes and node cards to freeze at zero previously while
    # the chart (fed from `history`, appended above) kept moving.
    try:
        persist_row(
            timestamp, raw_count, confirmed_count, candidate_count,
            node_counts, mean_rssi, rssi_stddev,
            total_probes, confidence, round(agreement * 100, 1)
        )
    except Exception as exc:
        print("[DB WRITE ERROR - continuing without persistence this cycle]", exc)

    with lock:
        latest.update({
            "room_count": room_count,
            "raw_count": raw_count,
            "candidate_count": candidate_count,
            "confidence": confidence,
            "display_mode": current_display_mode,
            "mean_rssi": round(mean_rssi, 1),
            "rssi_stddev": round(rssi_stddev, 1),
            "node_agreement": round(agreement * 100, 1),
            "probe_rate": total_probes,
            "unusual_jump": unusual_jump,
            "jump_score": jump_score,
            "per_node_status": per_node_status,
            "fused_devices": fused,
            "rssi_threshold": RSSI_PRESENCE_THRESHOLD,
            "last_update": timestamp,
        })


# =============================================================
# BACKGROUND LOOP
# =============================================================

def poll_loop():
    while True:
        try:
            poll_once()
        except Exception as exc:
            print("[POLL ERROR]", exc)
        time.sleep(POLL_INTERVAL_S)


# =============================================================
# API
# =============================================================

@app.route("/api/fused")
def api_fused():
    with lock:
        result = dict(latest)
        result["history"] = list(history)[-300:]
        return jsonify(result)


@app.route("/api/history")
def api_history():
    with lock:
        return jsonify({"history": list(history)})


# =============================================================
# DASHBOARD
# =============================================================

PAGE = r"""
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Occupancy Intelligence</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
:root {
    --bg: #080b0f; --panel: #10151b; --panel2: #151b22; --border: #222b34;
    --text: #e8eef2; --muted: #71808c;
    --cyan: #4fd8e8; --green: #57e389; --yellow: #ffb000; --red: #ff5252;
}
* { box-sizing: border-box; }
body {
    margin: 0;
    background: radial-gradient(circle at top right, #12212a 0, var(--bg) 40%);
    color: var(--text);
    font-family: Inter, ui-sans-serif, system-ui, -apple-system, sans-serif;
}
.container { max-width: 1500px; margin: auto; padding: 28px; }
.header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 25px; flex-wrap: wrap; gap: 14px; }
.title { font-size: 22px; font-weight: 700; letter-spacing: 0.08em; }
.subtitle { color: var(--muted); font-size: 12px; margin-top: 5px; }
.system { display: flex; align-items: center; gap: 9px; font-size: 12px; }
.dot { width: 9px; height: 9px; border-radius: 50%; background: var(--green); box-shadow: 0 0 12px var(--green); }
.dot.bad { background: var(--red); box-shadow: 0 0 12px var(--red); }

.infobar {
    display: flex; align-items: center; gap: 18px;
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 10px 14px; margin-bottom: 18px; flex-wrap: wrap;
    font-size: 12px; color: var(--muted);
}
.infobar b { color: var(--cyan); }

.metrics { display: grid; grid-template-columns: 1.5fr 1fr 1fr 1fr; gap: 14px; }
.card { background: linear-gradient(145deg, var(--panel2), var(--panel)); border: 1px solid var(--border); border-radius: 14px; padding: 20px; }
.card-label { color: var(--muted); text-transform: uppercase; font-size: 10px; letter-spacing: 0.12em; }
.big { font-size: 62px; line-height: 1; margin-top: 12px; color: var(--cyan); font-weight: 700; }
.metric { font-size: 28px; margin-top: 13px; font-weight: 600; }
.good { color: var(--green); } .warn { color: var(--yellow); } .bad { color: var(--red); }

.grid { display: grid; grid-template-columns: 2fr 1fr; gap: 14px; margin-top: 14px; }
.chart { height: 390px; }
.nodes { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; margin-top: 14px; }
.node { min-height: 150px; }
.node-header { display: flex; justify-content: space-between; align-items: center; }
.node-name { font-weight: 700; letter-spacing: 0.05em; }
.node-status { font-size: 10px; padding: 4px 8px; border-radius: 20px; background: #183020; color: var(--green); }
.node-status.offline { background: #351919; color: var(--red); }
.node-count { font-size: 38px; margin-top: 20px; }
.node-meta { display: flex; justify-content: space-between; color: var(--muted); font-size: 11px; margin-top: 8px; }

.stats { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 15px; }
.stat { background: #0b1015; padding: 12px; border-radius: 9px; }
.stat-name { font-size: 10px; color: var(--muted); }
.stat-value { margin-top: 5px; font-size: 18px; }

.devices { max-height: 280px; overflow: auto; margin-top: 14px; }
.device { display: flex; justify-content: space-between; padding: 8px 4px; border-bottom: 1px solid #192027; font-size: 11px; }
.device-id { color: var(--muted); }
.badge { padding: 3px 7px; border-radius: 6px; background: #18232a; color: var(--cyan); }

.footer { margin-top: 18px; color: var(--muted); font-size: 10px; display: flex; justify-content: space-between; flex-wrap: wrap; gap: 8px; }
.footer a { color: var(--cyan); text-decoration: none; }

@media(max-width: 1000px) { .metrics { grid-template-columns: repeat(2, 1fr); } .grid { grid-template-columns: 1fr; } .nodes { grid-template-columns: 1fr; } }
@media(max-width: 600px) { .container { padding: 15px; } .metrics { grid-template-columns: 1fr; } .header { align-items: flex-start; } }
</style>
</head>
<body>
<div class="container">

<div class="header">
  <div>
    <div class="title">OCCUPANCY INTELLIGENCE</div>
    <div class="subtitle">MULTI-NODE WIFI SENSOR FUSION</div>
  </div>
  <div class="system">
    <div id="systemDot" class="dot"></div>
    <div id="systemText">INITIALIZING</div>
  </div>
</div>

<div class="infobar">
  <span>RSSI presence gate: <b id="rssiThreshold">--</b> dBm (weaker signals treated as bleed-through)</span>
  <span>Display mode: <b id="displayModeChip">--</b> (double-clap on the control strip to cycle)</span>
</div>

<div class="metrics">
  <div class="card">
    <div class="card-label">Confirmed Occupancy</div>
    <div class="big" id="occupancy">--</div>
    <div class="subtitle">devices seen consistently, RSSI-gated</div>
  </div>
  <div class="card">
    <div class="card-label">Confidence</div>
    <div class="metric good" id="confidence">--</div>
    <div class="subtitle">sensor quality score</div>
  </div>
  <div class="card">
    <div class="card-label">Candidates</div>
    <div class="metric" id="candidates">--</div>
    <div class="subtitle">seen, not yet confirmed</div>
  </div>
  <div class="card">
    <div class="card-label">Unusual Jump</div>
    <div class="metric" id="anomaly">NORMAL</div>
    <div class="subtitle" id="anomalyScore">score: --</div>
  </div>
</div>

<div class="grid">
  <div class="card">
    <div class="card-label">Occupancy History</div>
    <div class="chart"><canvas id="occupancyChart"></canvas></div>
  </div>
  <div class="card">
    <div class="card-label">Sensor Stats</div>
    <div class="stats" style="margin-top:8px;">
      <div class="stat"><div class="stat-name">NODE AGREEMENT</div><div class="stat-value" id="agreement">--</div></div>
      <div class="stat"><div class="stat-name">MEAN RSSI</div><div class="stat-value" id="rssi">--</div></div>
      <div class="stat"><div class="stat-name">RSSI VARIANCE</div><div class="stat-value" id="rssiStd">--</div></div>
      <div class="stat"><div class="stat-name">DETECTION RATE</div><div class="stat-value" id="probeRate">--</div></div>
    </div>
  </div>
</div>

<div class="nodes" id="nodes"></div>

<div class="card" style="margin-top:14px;">
  <div class="card-label">Fused Sensor Observations</div>
  <div class="devices" id="devices"></div>
</div>

<div class="footer">
  <div id="lastUpdate">Last update: --</div>
  <div>ESP32 SENSOR FUSION ENGINE</div>
</div>

</div>

<script>
let chart;
let chartNodeNames = [];

const NODE_COLOR_PALETTE = ["#ffb000", "#57e389", "#ff5252", "#a78bfa", "#4fd8e8"];

function nodeColor(index) {
    return NODE_COLOR_PALETTE[index % NODE_COLOR_PALETTE.length];
}

function initializeChart() {
    const ctx = document.getElementById("occupancyChart").getContext("2d");
    chart = new Chart(ctx, {
        type: "line",
        data: {
            labels: [],
            datasets: [{
                label: "Confirmed (fused)",
                data: [],
                borderWidth: 3,
                borderColor: "#4fd8e8",
                backgroundColor: "rgba(79,216,232,0.15)",
                fill: true,
                tension: 0.35,
                pointRadius: 0,
                order: 0,
            }]
        },
        options: {
            responsive: true, maintainAspectRatio: false, animation: false,
            plugins: {
                legend: {
                    display: true,
                    labels: { color: "#71808c", boxWidth: 12, font: { size: 10 } }
                }
            },
            scales: {
                x: { ticks: { color: "#71808c", maxTicksLimit: 8 }, grid: { color: "#172027" } },
                y: { beginAtZero: true, ticks: { color: "#71808c" }, grid: { color: "#172027" } }
            }
        }
    });
}

function ensureNodeDatasets(nodeNames) {
    // Only rebuild if the set of known node names actually changed --
    // avoids recreating datasets (and losing their draw state) every poll.
    const changed = nodeNames.length !== chartNodeNames.length ||
        nodeNames.some((n, i) => n !== chartNodeNames[i]);

    if (!changed) return;

    chartNodeNames = nodeNames.slice();

    // Dataset 0 is always the master fused line; datasets 1..N are per-node.
    chart.data.datasets = [chart.data.datasets[0]];

    nodeNames.forEach((name, i) => {
        chart.data.datasets.push({
            label: name,
            data: [],
            borderWidth: 1.5,
            borderColor: nodeColor(i),
            backgroundColor: "transparent",
            fill: false,
            tension: 0.35,
            pointRadius: 0,
            borderDash: [4, 2],
            order: 1,
        });
    });
}

function updateChart(history, nodeNames) {
    if (!chart) return;

    ensureNodeDatasets(nodeNames);

    const recent = history.slice(-180);
    chart.data.labels = recent.map(x => new Date(x.timestamp * 1000).toLocaleTimeString([], {hour:"2-digit",minute:"2-digit",second:"2-digit"}));
    chart.data.datasets[0].data = recent.map(x => x.occupancy);

    chartNodeNames.forEach((name, i) => {
        chart.data.datasets[i + 1].data = recent.map(x => {
            const byName = x.node_counts_by_name || {};
            const v = byName[name];
            return (v === null || v === undefined) ? null : v; // null -> gap, not a false drop to 0
        });
    });

    chart.update();
}

function updateDashboard(d) {
    document.getElementById("occupancy").textContent = d.room_count;
    document.getElementById("confidence").textContent = d.confidence + "%";
    document.getElementById("candidates").textContent = d.candidate_count;
    document.getElementById("rssiThreshold").textContent = d.rssi_threshold;
    document.getElementById("displayModeChip").textContent = (d.display_mode || "--").toUpperCase();

    const anomaly = document.getElementById("anomaly");
    anomaly.textContent = d.unusual_jump ? "DETECTED" : "NORMAL";
    anomaly.className = "metric " + (d.unusual_jump ? "bad" : "good");
    document.getElementById("anomalyScore").textContent = "score: " + d.jump_score;

    document.getElementById("agreement").textContent = d.node_agreement + "%";
    document.getElementById("rssi").textContent = d.mean_rssi + " dBm";
    document.getElementById("rssiStd").textContent = d.rssi_stddev + " dB";
    document.getElementById("probeRate").textContent = d.probe_rate;

    const statuses = Object.values(d.per_node_status);
    const online = statuses.filter(x => x.ok).length;
    const systemDot = document.getElementById("systemDot");
    const systemText = document.getElementById("systemText");

    if (online === statuses.length) {
        systemDot.className = "dot";
        systemText.textContent = "ALL SYSTEMS ONLINE";
    } else {
        systemDot.className = "dot bad";
        systemText.textContent = online + "/" + statuses.length + " NODES ONLINE";
    }

    const nodes = document.getElementById("nodes");
    nodes.innerHTML = "";
    for (const [name, s] of Object.entries(d.per_node_status)) {
        const div = document.createElement("div");
        div.className = "card node";
        div.innerHTML = `
            <div class="node-header">
                <div class="node-name">${name}</div>
                <div class="node-status ${s.ok ? "" : "offline"}">${s.ok ? "ONLINE" : "OFFLINE"}</div>
            </div>
            <div class="node-count">${s.ok ? s.devices : "--"}</div>
            <div class="node-meta"><span>detected devices</span><span>${s.ok ? "ACTIVE" : "NO DATA"}</span></div>
        `;
        nodes.appendChild(div);
    }

    const devices = document.getElementById("devices");
    devices.innerHTML = "";
    if (!d.fused_devices.length) {
        devices.innerHTML = `<div class="subtitle">No active observations</div>`;
    } else {
        for (const dev of d.fused_devices) {
            const row = document.createElement("div");
            row.className = "device";
            row.innerHTML = `
                <span class="device-id">${dev.id}</span>
                <span>${dev.rssi} dBm</span>
                <span class="badge">${dev.seen_by.length} node${dev.seen_by.length !== 1 ? "s" : ""}</span>
            `;
            devices.appendChild(row);
        }
    }

    updateChart(d.history || [], Object.keys(d.per_node_status || {}).sort());

    if (d.last_update) {
        document.getElementById("lastUpdate").textContent = "Last update: " + new Date(d.last_update * 1000).toLocaleTimeString();
    }
}

async function refresh() {
    try {
        const response = await fetch("/api/fused", { cache: "no-store" });
        const data = await response.json();
        updateDashboard(data);
    } catch (error) {
        document.getElementById("systemText").textContent = "SERVER ERROR";
        document.getElementById("systemDot").className = "dot bad";
    }
}

initializeChart();
refresh();
setInterval(refresh, 2000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


# =============================================================
# MAIN
# =============================================================

if __name__ == "__main__":
    print()
    print("=" * 60)
    print(" OCCUPANCY INTELLIGENCE SERVER (v4)")
    print("=" * 60)
    print()
    print("Nodes:")
    for name, url in NODES.items():
        print(f"  {name:<10} {url}")
    print()
    print("Database:", DB_PATH, "(diagnostic history, no calibration labels)")
    print("Dashboard: http://localhost:8000")
    print(f"Tuning knobs: RSSI_PRESENCE_THRESHOLD={RSSI_PRESENCE_THRESHOLD}, "
          f"CONFIRM_AFTER_POLLS={CONFIRM_AFTER_POLLS}, DROP_AFTER_MISSED_POLLS={DROP_AFTER_MISSED_POLLS}")
    print()

    init_db()

    thread = threading.Thread(target=poll_loop, daemon=True)
    thread.start()

    app.run(host="0.0.0.0", port=8000, threaded=True)