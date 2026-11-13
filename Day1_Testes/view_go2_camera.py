#!/usr/bin/env python3
"""
Unitree Go2 Live Camera Stream & AprilTag HUD Viewer
===================================================
Streams live 120° wide-angle camera frames directly from the Unitree Go2 chassis:
- Displays live OpenCV window with real-time AprilTag (tag36h11) & ArUco detection.
- Displays FPS, resolution, detected tag IDs, distance (m), and lateral angles.
- Provides a built-in lightweight HTTP MJPEG stream (http://localhost:8089) so you
  can also view the live camera stream in any web browser!
- Press 'q' or 'ESC' to exit the OpenCV window.
- Press 's' to save a calibration snapshot.

Usage:
    ./view_go2_camera.py [--net-if en11] [--port 8089] [--no-gui]
"""

import os
import sys
import time
import argparse
import threading
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler

# Add project root
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

VENV_SITE = os.path.join(PROJECT_ROOT, ".venv_realsense", "lib", "python3.14", "site-packages")
if os.path.exists(VENV_SITE) and VENV_SITE not in sys.path:
    sys.path.insert(0, VENV_SITE)

latest_jpeg_frame = None
frame_lock = threading.Lock()
running = True

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global latest_jpeg_frame, running
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Unitree Go2 Live Camera HUD</title>
                <style>
                    body { background: #121214; color: #fff; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; text-align: center; margin: 0; padding: 20px; }
                    h2 { margin: 10px 0; color: #3b82f6; }
                    .stream-box { display: inline-block; background: #000; border: 2px solid #2563eb; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 20px rgba(0,0,0,0.6); }
                    img { max-width: 90vw; height: auto; display: block; }
                    .badge { display: inline-block; padding: 4px 10px; margin: 6px; background: #1f2937; border-radius: 4px; font-size: 13px; border: 1px solid #374151; }
                </style>
            </head>
            <body>
                <h2>🐕 Unitree Go2 120° Wide-Angle Front Camera Stream</h2>
                <div>
                    <span class="badge">Stream: Active</span>
                    <span class="badge">Detector: AprilTag tag36h11</span>
                    <span class="badge">Resolution: 1280x720</span>
                </div>
                <div class="stream-box">
                    <img src="/stream.mjpg" />
                </div>
            </body>
            </html>
            """
            self.wfile.write(html.encode("utf-8"))
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=--frameboundary")
            self.end_headers()
            while running:
                with frame_lock:
                    frame = latest_jpeg_frame
                if frame is not None:
                    try:
                        self.wfile.write(b"--frameboundary\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break
                time.sleep(0.033)
        else:
            self.send_error(404)

def run_web_server(port: int = 8089):
    server = HTTPServer(("0.0.0.0", port), MJPEGHandler)
    server.serve_forever()

def main():
    parser = argparse.ArgumentParser(description="Unitree Go2 Live Camera Stream & AprilTag HUD")
    parser.add_argument("--net-if", type=str, default="en11", help="Network interface")
    parser.add_argument("--port", type=int, default=8089, help="HTTP MJPEG stream port")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless mode (browser view only)")
    parser.add_argument("--tag-size", type=float, default=0.16, help="Tag size in meters (default 0.16m for A4)")
    args = parser.parse_args()

    print("=" * 75)
    print("📹 INITIALIZING UNITREE GO2 120° FRONT CAMERA STREAM")
    print("=" * 75)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    print(f"  • Connecting to CycloneDDS on interface '{args.net_if}'...")
    ChannelFactoryInitialize(0, args.net_if)

    vc = VideoClient()
    vc.SetTimeout(3.0)
    vc.Init()

    # AprilTag Detector setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    aruco_params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    elif hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    else:
        detector = None

    # Load 120 fisheye calibration if available
    calib_file = os.path.join(PROJECT_ROOT, "config", "go2_fisheye_120_calib.npz")
    camera_matrix = None
    dist_coeffs = None
    if os.path.exists(calib_file):
        try:
            data = np.load(calib_file)
            camera_matrix = data["K"]
            dist_coeffs = data["D"]
            print(f"  ✓ Loaded calibrated 120° fisheye intrinsics from {os.path.basename(calib_file)}")
        except Exception:
            pass

    # Start MJPEG HTTP server thread
    web_thread = threading.Thread(target=run_web_server, args=(args.port,), daemon=True)
    web_thread.start()
    print(f"\n🌐 Web Browser Stream Available: http://localhost:{args.port}")
    if not args.no_gui:
        print("🖥️ Native OpenCV GUI window opening... (Press 'q' to quit, 's' to snapshot)\n")

    global latest_jpeg_frame, running
    fps_history = []
    t_prev = time.time()
    snapshot_count = 0

    try:
        while True:
            t0 = time.time()
            code, data = vc.GetImageSample()
            if code != 0 or not data or len(data) == 0:
                time.sleep(0.02)
                continue

            frame = cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            h, w = frame.shape[:2]
            if camera_matrix is None:
                # Default 120 FOV approximation for 1280x720
                fx = 550.0
                fy = 550.0
                cx = w / 2.0
                cy = h / 2.0
                camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
                dist_coeffs = np.zeros((4, 1), dtype=np.float32)

            # Detect AprilTags
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if detector:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

            detected_tags = []
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))
                for i, tag_id in enumerate(ids.flatten()):
                    c = corners[i][0]
                    # Estimate pose using solvePnP
                    half_s = args.tag_size / 2.0
                    obj_pts = np.array([
                        [-half_s,  half_s, 0],
                        [ half_s,  half_s, 0],
                        [ half_s, -half_s, 0],
                        [-half_s, -half_s, 0]
                    ], dtype=np.float32)
                    success, rvec, tvec = cv2.solvePnP(obj_pts, c, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if success:
                        dist_m = float(np.linalg.norm(tvec))
                        angle_deg = float(np.degrees(np.arctan2(tvec[0][0], tvec[2][0])))
                        detected_tags.append((int(tag_id), dist_m, angle_deg))
                        # Draw 3D axis
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.7, 2)
                        # Text badge
                        center_x = int(np.mean(c[:, 0]))
                        center_y = int(np.mean(c[:, 1]))
                        badge = f"ID:{tag_id} | {dist_m:.2f}m | {angle_deg:+.1f}deg"
                        cv2.putText(frame, badge, (center_x - 60, center_y - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Compute FPS
            dt = time.time() - t_prev
            t_prev = time.time()
            if dt > 0:
                fps_history.append(1.0 / dt)
                if len(fps_history) > 20:
                    fps_history.pop(0)
            avg_fps = np.mean(fps_history) if fps_history else 0.0

            # Draw HUD bar
            hud_bg = frame.copy()
            cv2.rectangle(hud_bg, (0, 0), (w, 42), (18, 18, 20), -1)
            cv2.addWeighted(hud_bg, 0.75, frame, 0.25, 0, frame)

            status_str = f"Unitree Go2 120deg Cam | {w}x{h} @ {avg_fps:.1f} FPS | Detected Tags: {len(detected_tags)}"
            cv2.putText(frame, status_str, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (50, 255, 50), 2)

            # Encode JPEG for Web stream
            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with frame_lock:
                latest_jpeg_frame = jpeg.tobytes()

            # Local OpenCV GUI
            if not args.no_gui:
                cv2.imshow("Unitree Go2 Live Camera HUD (AprilTag tag36h11)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                elif key == ord('s'):
                    snapshot_count += 1
                    s_name = f"Day1_Testes/go2_cam_snap_{snapshot_count:03d}.jpg"
                    cv2.imwrite(s_name, frame)
                    print(f"  📸 Saved camera snapshot to {s_name}")

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if not args.no_gui:
            cv2.destroyAllWindows()
        print("\n👋 Camera stream closed.")

if __name__ == "__main__":
    main()
