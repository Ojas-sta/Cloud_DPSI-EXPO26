#!/usr/bin/env python3
"""
Live Cockpit Visualizer for Direct Unitree Go2 Python Architecture
Features:
  • Real-time composite 720p HUD (Camera feed + Overhead Mini-Map + Telemetry Gauges + Subtitles)
  • Embedded HTTP MJPEG Streamer (Open in Chrome/Safari at http://localhost:8088)
  • Optional high-speed native cv2.imshow window
  • Milestone screenshot capture to artifacts directory
"""

import cv2
import numpy as np
import time
import math
import os
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Dict, Any, Optional, List, Tuple

ARTIFACT_DIR = "/Users/roopalisingh/.gemini/antigravity/brain/ab9c06fe-2bf2-446d-becc-2e4354fd5ec7"


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """Serves high-framerate MJPEG video stream to any web browser."""
    visualizer_ref = None

    def log_message(self, format, *args):
        # Suppress noisy HTTP request logs in console
        return

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Unitree Go2 Pro - Live Vision & Telemetry Cockpit</title>
                <style>
                    * { box-sizing: border-box; }
                    body {
                        margin: 0;
                        padding: 16px;
                        background: #090d16;
                        color: #f1f5f9;
                        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
                        display: flex;
                        flex-direction: column;
                        align-items: center;
                        min-height: 100vh;
                    }
                    .header {
                        width: 100%;
                        max-width: 1280px;
                        display: flex;
                        justify-content: space-between;
                        align-items: center;
                        margin-bottom: 12px;
                        padding: 12px 20px;
                        background: #111827;
                        border: 1px solid #1e293b;
                        border-radius: 12px;
                    }
                    .title {
                        font-size: 18px;
                        font-weight: 700;
                        color: #00e5ff;
                        letter-spacing: 0.5px;
                        display: flex;
                        align-items: center;
                        gap: 8px;
                    }
                    .status-pill {
                        background: rgba(16, 185, 129, 0.15);
                        border: 1px solid #10b981;
                        padding: 4px 12px;
                        border-radius: 999px;
                        font-size: 12px;
                        font-weight: 600;
                        color: #10b981;
                        display: flex;
                        align-items: center;
                        gap: 6px;
                    }
                    .pulse {
                        width: 8px;
                        height: 8px;
                        border-radius: 50%;
                        background: #10b981;
                        box-shadow: 0 0 10px #10b981;
                    }
                    .player-container {
                        width: 100%;
                        max-width: 1280px;
                        background: #000;
                        border-radius: 12px;
                        overflow: hidden;
                        border: 1px solid #1e293b;
                        box-shadow: 0 20px 50px rgba(0, 0, 0, 0.7);
                    }
                    img.stream {
                        width: 100%;
                        height: auto;
                        display: block;
                    }
                    .controls-bar {
                        width: 100%;
                        max-width: 1280px;
                        margin-top: 12px;
                        display: flex;
                        flex-wrap: wrap;
                        gap: 10px;
                        justify-content: center;
                        background: #111827;
                        padding: 12px;
                        border-radius: 12px;
                        border: 1px solid #1e293b;
                    }
                    .btn {
                        background: #1e293b;
                        color: #e2e8f0;
                        border: 1px solid #334155;
                        padding: 8px 16px;
                        border-radius: 8px;
                        font-size: 13px;
                        font-weight: 600;
                        cursor: pointer;
                        display: flex;
                        align-items: center;
                        gap: 6px;
                        transition: all 0.15s ease;
                    }
                    .btn:hover {
                        background: #334155;
                        border-color: #00e5ff;
                        color: #fff;
                    }
                    .btn:active {
                        transform: scale(0.97);
                    }
                    .footer {
                        margin-top: 14px;
                        font-size: 13px;
                        color: #64748b;
                        text-align: center;
                    }
                </style>
            </head>
            <body>
                <div class="header">
                    <div class="title">
                        <span>🤖</span> UNITREE GO2 PRO • DIRECT PYTHON VISION COCKPIT
                    </div>
                    <div class="status-pill">
                        <span class="pulse"></span> LIVE MJPEG STREAM (30 FPS)
                    </div>
                </div>
                <div class="player-container">
                    <img class="stream" src="/stream.mjpg" alt="Unitree Go2 Live Camera Stream" />
                </div>
                <!-- Interactive Evaluator & Topological SLAM Controls -->
                <div class="controls-bar">
                    <button class="btn" onclick="sendCmd('/api/mode/tour')">🚀 Mode: Tour</button>
                    <button class="btn" onclick="sendCmd('/api/mode/discovery')">🗺️ Mode: Discovery</button>
                    <button class="btn" onclick="sendCmd('/api/save_graph')">💾 Save Graph (M)</button>
                    <button class="btn" onclick="sendCmd('/api/judge/lag')">⏪ Judges Lag ([)</button>
                    <button class="btn" onclick="sendCmd('/api/judge/lead')">⏩ Judges Lead (])</button>
                    <button class="btn" onclick="sendCmd('/api/judge/toggle_walk')">⏸️ Pause Judges (P)</button>
                    <button class="btn" onclick="sendCmd('/api/judge/toggle_flank')">↔️ Flank (\\)</button>
                </div>
                <!-- Interactive Keyboard & On-Screen Teleop D-Pad -->
                <div class="controls-bar" style="background: #0f172a; border-color: #3b82f6;">
                    <span style="font-size: 13px; font-weight: 700; color: #38bdf8; display: flex; align-items: center; margin-right: 8px;">🎮 TELEOP:</span>
                    <button class="btn" onclick="sendCmd('/api/teleop/forward')">▲ Fwd (W)</button>
                    <button class="btn" onclick="sendCmd('/api/teleop/backward')">▼ Rev (S)</button>
                    <button class="btn" onclick="sendCmd('/api/teleop/strafe_left')">◀ Left (A)</button>
                    <button class="btn" onclick="sendCmd('/api/teleop/strafe_right')">▶ Right (D)</button>
                    <button class="btn" onclick="sendCmd('/api/teleop/turn_left')">↶ Turn L (Q)</button>
                    <button class="btn" onclick="sendCmd('/api/teleop/turn_right')">↷ Turn R (E)</button>
                    <button class="btn" style="border-color: #ef4444; color: #fca5a5;" onclick="sendCmd('/api/teleop/brake')">🛑 STOP (Space)</button>
                </div>
                <div class="footer">
                    DPSI EXPO-2026 Autonomous Docent Mission • 360° LiDAR Companion Pacing & Real OpenCV Perception
                </div>
                <script>
                    function sendCmd(endpoint) {
                        fetch(endpoint, { method: 'POST' }).catch(() => {});
                    }
                    window.addEventListener('keydown', (e) => {
                        const k = e.key.toLowerCase();
                        if (e.key === '[') sendCmd('/api/judge/lag');
                        else if (e.key === ']') sendCmd('/api/judge/lead');
                        else if (k === 'p') sendCmd('/api/judge/toggle_walk');
                        else if (e.key === '\\\\') sendCmd('/api/judge/toggle_flank');
                        else if (k === 'w') sendCmd('/api/teleop/forward');
                        else if (k === 's') sendCmd('/api/teleop/backward');
                        else if (k === 'a') sendCmd('/api/teleop/strafe_left');
                        else if (k === 'd') sendCmd('/api/teleop/strafe_right');
                        else if (k === 'q') sendCmd('/api/teleop/turn_left');
                        else if (k === 'e') sendCmd('/api/teleop/turn_right');
                        else if (e.key === ' ') sendCmd('/api/teleop/brake');
                        else if (k === 'm') sendCmd('/api/save_graph');
                    });
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    if MJPEGStreamHandler.visualizer_ref is not None:
                        frame_bytes = MJPEGStreamHandler.visualizer_ref.get_latest_jpeg()
                        if frame_bytes is not None:
                            self.wfile.write(b'--FRAME\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(frame_bytes)))
                            self.end_headers()
                            self.wfile.write(frame_bytes)
                            self.wfile.write(b'\r\n')
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError, Exception):
                pass
        else:
            self.send_error(404, "File Not Found")

    def do_POST(self):
        """Handles interactive hotkeys, teleop, and mode toggles from web buttons."""
        vis = MJPEGStreamHandler.visualizer_ref
        if vis is not None:
            # Mode switches
            if self.path == '/api/mode/discovery':
                vis.requested_mode = "discovery"
                print("  🌐 [Web UI]: Mode switched to DISCOVERY MAPPER")
            elif self.path == '/api/mode/tour':
                vis.requested_mode = "tour"
                print("  🌐 [Web UI]: Mode switched to TOPOLOGICAL TOUR")
            elif self.path == '/api/save_graph':
                if vis.discovery_ref:
                    vis.discovery_ref.save_discovered_map()
                    print("  🌐 [Web UI]: Discovered topological graph saved!")
            # Teleop commands
            elif self.path.startswith('/api/teleop'):
                if vis.teleop_ref:
                    if self.path == '/api/teleop/forward':
                        vis.teleop_ref.inject_command(0.30, 0.0, 0.0)
                    elif self.path == '/api/teleop/backward':
                        vis.teleop_ref.inject_command(-0.25, 0.0, 0.0)
                    elif self.path == '/api/teleop/strafe_left':
                        vis.teleop_ref.inject_command(0.0, 0.20, 0.0)
                    elif self.path == '/api/teleop/strafe_right':
                        vis.teleop_ref.inject_command(0.0, -0.20, 0.0)
                    elif self.path == '/api/teleop/turn_left':
                        vis.teleop_ref.inject_command(0.0, 0.0, 0.75)
                    elif self.path == '/api/teleop/turn_right':
                        vis.teleop_ref.inject_command(0.0, 0.0, -0.75)
                    elif self.path == '/api/teleop/brake':
                        vis.teleop_ref.inject_command(0.0, 0.0, 0.0)
            # Evaluator pacing hotkeys
            elif vis.escort_ref:
                escort = vis.escort_ref
                if self.path == '/api/judge/lag':
                    escort.adjust_judge_offset(-0.15)
                elif self.path == '/api/judge/lead':
                    escort.adjust_judge_offset(+0.15)
                elif self.path == '/api/judge/toggle_walk':
                    escort.toggle_judge_walking()
                elif self.path == '/api/judge/toggle_flank':
                    escort.toggle_judge_flank()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')


class CockpitVisualizer:
    def __init__(self, port: int = 8088, enable_native_window: bool = False, active_milestone: Optional[str] = "M1"):
        self.port = port
        self.enable_native_window = enable_native_window
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_jpeg: Optional[bytes] = None
        self.lock = threading.Lock()
        self.escort_ref = None
        self.discovery_ref = None
        self.navigator_ref = None
        self.teleop_ref = None
        self.active_milestone: Optional[str] = f"M{active_milestone}" if str(active_milestone).isdigit() else active_milestone
        self.requested_mode: Optional[str] = None

        # Start multi-threaded HTTP MJPEG stream server
        MJPEGStreamHandler.visualizer_ref = self
        self.server = None
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()
        print(f"  🌐 [CockpitVisualizer]: Multi-threaded Web Visualizer listening at http://localhost:{self.port}")

    def _run_server(self):
        try:
            self.server = ThreadingHTTPServer(('0.0.0.0', self.port), MJPEGStreamHandler)
            self.server.serve_forever()
        except Exception as e:
            print(f"  ⚠️ [CockpitVisualizer]: HTTP server notice: {e}")

    def get_latest_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return self.latest_jpeg

    def render_overhead_minimap(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw_deg: float,
        escort_info: Optional[Dict[str, Any]] = None,
        graph_nodes: Optional[Dict[int, Dict[str, Any]]] = None,
        graph_edges: Optional[List[Dict[str, Any]]] = None,
        active_route: Optional[List[int]] = None
    ) -> np.ndarray:
        """Renders an overhead 2D tactical radar map (280x280) with topological ArUco pose graph."""
        m = np.full((280, 280, 3), (18, 15, 12), dtype=np.uint8)

        # Scale & center: Exhibition concourse from Y=7.0 to Y=15.0, X=-2.0 to 4.0
        # Map pixels: 280x280 -> X: [-2.0, 4.0], Y: [7.0, 15.0]
        def to_px(x, y):
            px = int(140 + (x - 0.0) * 38.0)
            py = int(260 - (y - 7.5) * 32.0)
            return px, py

        # Concourse Arterial Track
        pt1 = to_px(0.0, 7.5)
        pt2 = to_px(0.0, 14.5)
        cv2.line(m, pt1, pt2, (45, 40, 35), 18, cv2.LINE_AA)
        cv2.line(m, pt1, pt2, (0, 220, 255), 1, cv2.LINE_AA)

        # Draw Topological Pose Graph if available
        if graph_nodes:
            # Draw edges first
            if graph_edges:
                for e in graph_edges:
                    u, v = int(e["from"]), int(e["to"])
                    if u in graph_nodes and v in graph_nodes:
                        pu = to_px(graph_nodes[u]["pose"]["x"], graph_nodes[u]["pose"]["y"])
                        pv = to_px(graph_nodes[v]["pose"]["x"], graph_nodes[v]["pose"]["y"])
                        # Highlight active route edge
                        is_active_edge = False
                        if active_route and len(active_route) > 1:
                            for idx in range(len(active_route) - 1):
                                if (active_route[idx] == u and active_route[idx+1] == v) or (active_route[idx] == v and active_route[idx+1] == u):
                                    is_active_edge = True
                                    break
                        if is_active_edge:
                            cv2.line(m, pu, pv, (0, 255, 120), 3, cv2.LINE_AA)
                        else:
                            cv2.line(m, pu, pv, (80, 140, 200), 1, cv2.LINE_AA)

            # Draw nodes
            for nid, ndata in graph_nodes.items():
                nx_val = ndata["pose"]["x"]
                ny_val = ndata["pose"]["y"]
                npx, npy = to_px(nx_val, ny_val)
                is_on_route = active_route and nid in active_route

                if ndata.get("tag_type") == "FLOOR":
                    # Floor Tag: Cyan circle
                    col = (0, 255, 120) if is_on_route else (0, 220, 255)
                    cv2.circle(m, (npx, npy), 6, col, -1)
                    cv2.circle(m, (npx, npy), 10, col, 1)
                    cv2.putText(m, f"#{nid}", (npx - 10, npy + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (200, 240, 255), 1)
                else:
                    # Booth Tag: Amber rectangle
                    b_col = (0, 215, 255) if is_on_route else (255, 180, 0)
                    cv2.rectangle(m, (npx - 14, npy - 8), (npx + 14, npy + 8), b_col, 2)
                    lbl = ndata.get("label", f"#{nid}")[:3]
                    cv2.putText(m, lbl, (npx - 10, npy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        else:
            # Fallback floor & stall markers
            for fy in [9.0, 11.0, 13.0]:
                fpx, fpy = to_px(0.0, fy)
                cv2.circle(m, (fpx, fpy), 5, (0, 220, 255), -1)

            stalls = [("P01", 0.0, 14.0, (255, 180, 0)), ("P06", 0.0, 8.5, (0, 200, 255))]
            for name, sx, sy, col in stalls:
                spx, spy = to_px(sx, sy)
                cv2.rectangle(m, (spx - 18, spy - 10), (spx + 18, spy + 10), col, 2)
                cv2.putText(m, name, (spx - 14, spy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Draw Robot position & heading arrow (0°=East/Right, 90°=North/Up, -90°=South/Down)
        rx, ry = to_px(robot_x, robot_y)
        yaw_rad = math.radians(robot_yaw_deg)
        hx = int(rx + 24.0 * math.cos(yaw_rad))
        hy = int(ry - 24.0 * math.sin(yaw_rad))

        cv2.circle(m, (rx, ry), 9, (0, 255, 120), -1)
        cv2.circle(m, (rx, ry), 15, (0, 255, 120), 1)
        cv2.arrowedLine(m, (rx, ry), (hx, hy), (0, 255, 255), 2, tipLength=0.35)

        # Draw Judge Escort Companion Avatar if active
        if escort_info and escort_info.get("judge_tracked", True):
            dx = escort_info.get("judge_rel_x", 0.0)
            dy = escort_info.get("judge_rel_y", 0.90)
            jx = robot_x + dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
            jy = robot_y + dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
            jpx, jpy = to_px(jx, jy)

            cv2.line(m, (rx, ry), (jpx, jpy), (60, 160, 240), 1, cv2.LINE_AA)
            cv2.circle(m, (jpx, jpy), 7, (255, 180, 0), -1)
            cv2.circle(m, (jpx, jpy), 12, (255, 220, 50), 1)
            cv2.putText(m, "JUDGE", (jpx - 14, jpy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Title
        cv2.putText(m, "TOPOLOGICAL CONCOURSE RADAR", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 229, 255), 1)
        return m

    def compose_cockpit_frame(
        self,
        camera_annotated: np.ndarray,
        telemetry: Dict[str, Any],
        active_mission_stage: str,
        speech_text: str = "",
        escort_info: Optional[Dict[str, Any]] = None,
        mode_label: str = "TOPOLOGICAL TOUR",
        topological_info: Optional[Dict[str, Any]] = None
    ) -> np.ndarray:
        """Assembles the full 1280x720 broadcast HUD."""
        hud = camera_annotated.copy()

        # 1. Top Telemetry Header Bar
        cv2.rectangle(hud, (0, 0), (1280, 56), (15, 18, 25), -1)
        cv2.line(hud, (0, 56), (1280, 56), (0, 229, 255), 2)

        # Logo & Platform
        cv2.putText(hud, "UNITREE GO2 PRO", (20, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 229, 255), 2)
        m_tag = f" • [{self.active_milestone}]" if self.active_milestone else ""
        cv2.putText(hud, f"DIRECT SDK2 • {mode_label}{m_tag}", (20, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 180), 1)

        # Telemetry Pills
        posture = telemetry.get("posture", "STAND")
        pos_str = f"Pos: ({telemetry.get('x', 0.0):.2f}m, {telemetry.get('y', 0.0):.2f}m) | Yaw: {telemetry.get('yaw_deg', 0.0):.1f}°"
        batt_str = f"BATTERY: {telemetry.get('battery_pct', 90):.0f}% ({telemetry.get('voltage', 28.4):.1f}V)"
        pitch_str = f"Head Tilt: {telemetry.get('pitch_deg', 0.0):+.1f}°"

        cv2.putText(hud, f"STATE: {posture}", (360, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 120), 2)
        cv2.putText(hud, pos_str, (560, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (226, 232, 240), 1)
        cv2.putText(hud, pitch_str, (910, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 229, 255), 1)
        cv2.putText(hud, batt_str, (1080, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 120), 2)

        # 2. Top-Right Overhead Mini-Map Inset (Topological Graph Render)
        graph_nodes = None
        graph_edges = None
        active_route = None
        if topological_info:
            graph_nodes = topological_info.get("nodes")
            graph_edges = topological_info.get("edges")
            active_route = topological_info.get("active_route")

        minimap = self.render_overhead_minimap(
            telemetry.get("x", 0.0),
            telemetry.get("y", 11.5),
            telemetry.get("yaw_deg", 90.0),
            escort_info=escort_info,
            graph_nodes=graph_nodes,
            graph_edges=graph_edges,
            active_route=active_route
        )
        mh, mw, _ = minimap.shape
        hud[70:70+mh, 1280-mw-20:1280-20] = minimap
        cv2.rectangle(hud, (1280-mw-20, 70), (1280-20, 70+mh), (0, 229, 255), 1)

        # 3. Mission Phase Badge (Top Left of Camera View)
        badge_y = 90
        cv2.rectangle(hud, (20, badge_y), (520, badge_y + 36), (15, 23, 42), -1)
        cv2.rectangle(hud, (20, badge_y), (520, badge_y + 36), (0, 229, 255), 1)
        cv2.putText(hud, f"STAGE: {active_mission_stage}", (32, badge_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 229, 255), 2)

        # 4. Topological SLAM & Route Status Chip
        if topological_info:
            ty = badge_y + 44
            cv2.rectangle(hud, (20, ty), (520, ty + 34), (15, 23, 42), -1)
            cv2.rectangle(hud, (20, ty), (520, ty + 34), (0, 255, 120), 1)
            if mode_label == "DISCOVERY MAPPER":
                f_count = topological_info.get("floor_tags_count", 0)
                b_count = topological_info.get("booth_tags_count", 0)
                e_count = topological_info.get("total_edges", 0)
                top_str = f"DISCOVERED: {f_count} Floor Tags | {b_count} Booth Aprons | {e_count} Relative Edges"
            else:
                r_str = " -> ".join([f"#{t}" for t in (active_route or [])])
                top_str = f"ROUTE: {r_str if r_str else 'STANDBY'} | Reset: 0.00m Drift"
            cv2.putText(hud, top_str, (32, ty + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)

        # 5. Companion Judge Escort Telemetry Chip (if active)
        if escort_info:
            ey = badge_y + 86
            s_badge = escort_info.get("status_badge", "ESCORT: ACTIVE")
            s_color = escort_info.get("status_color", (0, 255, 120))
            flank = escort_info.get("flank", "LEFT")
            dx = escort_info.get("judge_rel_x", 0.0)
            dl = escort_info.get("d_left", 1.40)
            dr = escort_info.get("d_right", 1.60)
            cmd_vx = escort_info.get("vx", 0.0)
            cmd_vy = escort_info.get("vy", 0.0)

            # Escort Status Card
            cv2.rectangle(hud, (20, ey), (520, ey + 54), (15, 23, 42), -1)
            cv2.rectangle(hud, (20, ey), (520, ey + 54), s_color, 1)

            cv2.putText(hud, s_badge, (32, ey + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, s_color, 2)
            escort_sub = f"FLANK: {flank} | Δx: {dx:+.2f}m | L: {dl:.1f}m R: {dr:.1f}m | Cmd: [{cmd_vx:.2f}, {cmd_vy:+.2f}]"
            cv2.putText(hud, escort_sub, (32, ey + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (203, 213, 225), 1)

        # 6. Bottom Speech Transcript Banner
        if speech_text:
            cv2.rectangle(hud, (0, 640), (1280, 720), (11, 15, 25), -1)
            cv2.line(hud, (0, 640), (1280, 640), (0, 229, 255), 1)
            cv2.putText(hud, "[DOCENT SPEECH SYNTHESIS]", (20, 665), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 229, 255), 2)
            display_txt = speech_text[:115] + ("..." if len(speech_text) > 115 else "")
            cv2.putText(hud, display_txt, (20, 695), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        # Update latest frame & JPEG
        with self.lock:
            self.latest_frame = hud
            _, jpeg = cv2.imencode('.jpg', hud, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.latest_jpeg = jpeg.tobytes()

        if self.enable_native_window:
            cv2.imshow("Unitree Go2 - Direct Python Vision Cockpit", hud)
            cv2.waitKey(1)

        return hud
        if speech_text:
            cv2.rectangle(hud, (0, 640), (1280, 720), (11, 15, 25), -1)
            cv2.line(hud, (0, 640), (1280, 640), (0, 229, 255), 1)
            cv2.putText(hud, "[DOCENT SPEECH SYNTHESIS]", (20, 665), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 229, 255), 2)
            # Truncate text if too long
            display_txt = speech_text[:115] + ("..." if len(speech_text) > 115 else "")
            cv2.putText(hud, display_txt, (20, 695), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        # Update latest frame & JPEG
        with self.lock:
            self.latest_frame = hud
            _, jpeg = cv2.imencode('.jpg', hud, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self.latest_jpeg = jpeg.tobytes()

        if self.enable_native_window:
            cv2.imshow("Unitree Go2 - Direct Python Vision Cockpit", hud)
            cv2.waitKey(1)

        return hud

    def save_milestone_artifact(self, filename: str):
        """Saves current visualizer HUD snapshot to conversation artifacts directory."""
        with self.lock:
            if self.latest_frame is not None:
                full_path = os.path.join(ARTIFACT_DIR, filename)
                cv2.imwrite(full_path, self.latest_frame)
                print(f"    📸 [CockpitVisualizer]: Saved visual artifact -> {filename}")
