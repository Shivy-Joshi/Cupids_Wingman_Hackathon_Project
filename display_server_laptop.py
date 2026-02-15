#!/usr/bin/env python3
"""
display_server_laptop.py
Run on your laptop. Phone connects over the same Wi-Fi.

Setup:
  py -m pip install flask

Run:
  py display_server_laptop.py

Phone:
  Open one of the printed URLs in Chrome/Safari (same Wi-Fi).
"""

from flask import Flask, Response, request, jsonify
import socket

app = Flask(__name__)

STATE = {
    "mood": "neutral",   # neutral | happy | sleepy | alert
    "lookX": 0.0,        # -1.0 .. +1.0
    "lookY": 0.0,        # -1.0 .. +1.0
    "blink": 0.0         # 0.0 .. 1.0 (manual override if you want)
}

HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no" />
  <title>Robot Display</title>
  <style>
    html, body { margin:0; height:100%; background:#000; overflow:hidden; }
    canvas { display:block; width:100%; height:100%; }
    .hint {
      position:fixed; left:12px; bottom:12px; color:#777;
      font:14px system-ui, -apple-system, Segoe UI, Roboto;
      user-select:none;
    }
  </style>
</head>
<body>
  <canvas id="c"></canvas>
  <div class="hint">Phone Display • Tap for fullscreen</div>

<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

function resize(){
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(innerWidth * dpr);
  canvas.height = Math.floor(innerHeight * dpr);
  // Draw in CSS pixels while backing store is scaled by dpr
  ctx.setTransform(dpr,0,0,dpr,0,0);
}
addEventListener('resize', resize);
resize();

let state = { mood:"neutral", lookX:0, lookY:0, blink:0 };
let lastFetch = 0;

async function pollState(){
  try{
    const r = await fetch('/state', { cache: "no-store" });
    if(r.ok){
      const s = await r.json();
      state = Object.assign(state, s);
    }
  }catch(e){
    // ignore transient network errors
  }
}

function clamp(v, a, b){ return Math.max(a, Math.min(b, v)); }

function drawEyes(t){
  const W = innerWidth, H = innerHeight;

  // Background
  ctx.fillStyle = "#000";
  ctx.fillRect(0,0,W,H);

  const cy = H * 0.50;

  // Eye centers at 25% and 75%
  const leftCenterX  = W * 0.25;
  const rightCenterX = W * 0.75;

  // Asymmetric sizing: left larger, right smaller
  const baseW = Math.min(W * 0.18, H * 0.35);
  const leftW  = baseW * 1.18;
  const rightW = baseW * 0.92;

  const leftH  = leftW  * 1.8;
  const rightH = rightW * 1.8;

  // Idle + look offsets
  const idleX = Math.sin(t*0.0006) * 0.05;
  const idleY = Math.sin(t*0.0009) * 0.05;

  const lx = clamp(state.lookX + idleX, -1, 1);
  const ly = clamp(state.lookY + idleY, -1, 1);

  // --- Auto blink every ~3 seconds ---
  const BLINK_PERIOD_MS = 3000;
  const BLINK_WINDOW_MS = 140;
  const phase = t % BLINK_PERIOD_MS;

  let blinkAuto = 0;
  if (phase < BLINK_WINDOW_MS) {
    const x = phase / BLINK_WINDOW_MS; // 0..1
    const tri = x < 0.5 ? (x * 2) : (2 - x * 2); // 0..1..0
    // smoothstep(tri)
    blinkAuto = tri * tri * (3 - 2 * tri);
  }

  // Allow manual override via /state if you want
  const blink = clamp(Math.max(state.blink, blinkAuto), 0, 1);

  function roundedRect(x, y, w, h, r){
    ctx.beginPath();
    ctx.moveTo(x+r, y);
    ctx.arcTo(x+w, y, x+w, y+h, r);
    ctx.arcTo(x+w, y+h, x, y+h, r);
    ctx.arcTo(x, y+h, x, y, r);
    ctx.arcTo(x, y, x+w, y, r);
    ctx.closePath();
  }

  function drawOneEye(centerX, centerY, eyeW, eyeH, isLeft){
    const x = centerX - eyeW/2;
    const y = centerY - eyeH/2;

    const radius = eyeW * 0.45;

    // Subtle shadow for depth
    ctx.save();
    ctx.shadowColor = "rgba(0,0,0,0.35)";
    ctx.shadowBlur = 18;
    ctx.shadowOffsetY = 8;

    // White outer eye
    ctx.fillStyle = "#f7f7f7";
    roundedRect(x, y, eyeW, eyeH, radius);
    ctx.fill();
    ctx.restore();

    // Pupil proportions (slightly different per eye)
    const pupilW = eyeW * (isLeft ? 0.66 : 0.62);
    const pupilH = eyeH * (isLeft ? 0.78 : 0.72);

    const px = x + (eyeW - pupilW)/2 + lx * eyeW * 0.12;
    const py = y + (eyeH - pupilH)/2 + ly * eyeH * 0.15;

    ctx.fillStyle = "#4b1f1f";
    roundedRect(px, py, pupilW, pupilH, pupilW * 0.4);
    ctx.fill();

    // Blink: top + bottom lids
    if (blink > 0){
      const lid = blink * eyeH * 0.55;
      ctx.fillStyle = "#000";

      // top lid
      roundedRect(x-2, y-2, eyeW+4, lid, radius * 0.7);
      ctx.fill();

      // bottom lid
      roundedRect(x-2, y + eyeH - lid + 2, eyeW+4, lid, radius * 0.7);
      ctx.fill();
    }
  }

  drawOneEye(leftCenterX,  cy, leftW,  leftH,  true);
  drawOneEye(rightCenterX, cy, rightW, rightH, false);
}

function loop(t){
  // Poll state ~5x/sec
  if(t - lastFetch > 200){
    lastFetch = t;
    pollState();
  }
  drawEyes(t);
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

// Optional: tap for fullscreen (mobile browsers vary)
document.body.addEventListener('click', async () => {
  const el = document.documentElement;
  if(!document.fullscreenElement && el.requestFullscreen){
    try{ await el.requestFullscreen(); }catch(e){}
  }
});
</script>
</body>
</html>
"""

def get_candidate_ips():
    """List LAN-reachable IPv4 addresses for this machine."""
    ips = set()

    # UDP "connect" trick (no packets sent)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
    except Exception:
        pass
    finally:
        s.close()

    # Hostname resolution may add others
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ips.add(info[4][0])
    except Exception:
        pass

    ips.discard("127.0.0.1")
    return sorted(ips)

@app.get("/")
def index():
    return Response(HTML, mimetype="text/html")

@app.get("/state")
def get_state():
    return jsonify(STATE)

@app.post("/state")
def set_state():
    data = request.get_json(silent=True) or {}
    for k in ("mood", "lookX", "lookY", "blink"):
        if k in data:
            STATE[k] = data[k]
    return jsonify({"ok": True, "state": STATE})

if __name__ == "__main__":
    port = 5000
    print("\n=== Phone Display Server (Laptop) ===")
    ips = get_candidate_ips()
    if not ips:
        print(f"Could not auto-detect LAN IP. Try: http://<your-laptop-ip>:{port}/")
    else:
        print("Open ONE of these URLs on your phone (same Wi-Fi):")
        for ip in ips:
            print(f"  http://{ip}:{port}/")
    print("\nIf it doesn't load:")
    print("- Make sure phone + laptop are on the same Wi-Fi (not guest/AP isolation).")
    print("- Allow Python through your firewall for Private networks.\n")

    app.run(host="0.0.0.0", port=port, debug=False)