#!/usr/bin/env python3
"""
Interactive Non-Blocking Keyboard Teleoperation with 0.40s Auto-Zero Watchdog
Venue: DPSI EXPO-2026 Exhibition Hall | Platform: Unitree Go2 Edu / Pro Quadruped

Key Mappings:
  • W / S: Linear forward / reverse (+0.30 / -0.25 m/s)
  • A / D: Lateral left / right strafe (+0.20 / -0.20 m/s)
  • Q / E: Turn left / turn right (+0.75 / -0.75 rad/s)
  • Space: Full emergency brake (vx = vy = wz = 0)
  • M: Save discovered topological ArUco map & grid
  • X / Esc: Exit teleoperation mode

Refinement 3:
  Includes a 0.40-second auto-zero watchdog timer so that releasing keys or losing
  terminal focus smoothly decays motion commands to zero rather than running away.
"""

import sys
import time
import threading
import select
from typing import Optional, Callable, Tuple

# Platform check for POSIX termios
try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


class KeyboardTeleop:
    def __init__(
        self,
        command_callback: Optional[Callable[[float, float, float], None]] = None,
        save_map_callback: Optional[Callable[[], None]] = None,
        watchdog_timeout_sec: float = 0.40
    ):
        self.command_callback = command_callback
        self.save_map_callback = save_map_callback
        self.watchdog_timeout = watchdog_timeout_sec

        # Teleop state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = time.time()
        self.running = False
        self.listener_thread: Optional[threading.Thread] = None
        self.watchdog_thread: Optional[threading.Thread] = None

    def start(self):
        """Starts keyboard listener and watchdog threads."""
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._run_watchdog, daemon=True)
        self.watchdog_thread.start()

        if not HAS_TERMIOS:
            print("  ⚠️ [KeyboardTeleop]: termios not available or running non-interactively. Use Web Cockpit teleop buttons.")
            return

        self.listener_thread = threading.Thread(target=self._run_listener, daemon=True)
        self.listener_thread.start()

        print("\n" + "-" * 60)
        print("  🎮 [KeyboardTeleop]: INTERACTIVE TELEOP ACTIVE")
        print("  • Controls: W/S (Fwd/Rev) | A/D (Strafe) | Q/E (Turn) | Space (Brake)")
        print("  • Hotkey:   M (Save Discovered Map) | X (Exit)")
        print(f"  • Safety:   {self.watchdog_timeout:.2f}s Auto-Zero Command Watchdog")
        print("-" * 60 + "\n")

    def stop(self):
        """Stops listener and zeros velocities."""
        self.running = False
        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        if self.command_callback:
            self.command_callback(0.0, 0.0, 0.0)

    def check_watchdog(self) -> bool:
        """Refinement 3: Checks if watchdog timeout elapsed and zeroes velocity."""
        elapsed = time.time() - self.last_cmd_time
        if elapsed > self.watchdog_timeout:
            if self.vx != 0.0 or self.vy != 0.0 or self.wz != 0.0:
                self.vx = 0.0
                self.vy = 0.0
                self.wz = 0.0
                if self.command_callback:
                    self.command_callback(0.0, 0.0, 0.0)
                return True
        return False

    def _run_watchdog(self):
        """Background watchdog loop."""
        while self.running:
            time.sleep(0.05)
            self.check_watchdog()

    def _run_listener(self):
        """Captures single keypresses without requiring Enter."""
        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception as e:
            print(f"  ⚠️ [KeyboardTeleop]: Could not enable cbreak keyboard capture ({e}).")
            return

        try:
            while self.running:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    now = time.time()
                    self.last_cmd_time = now

                    if ch in ('w', 'W'):
                        self.vx = 0.30
                    elif ch in ('s', 'S'):
                        self.vx = -0.25
                    elif ch in ('a', 'A'):
                        self.vy = 0.20
                    elif ch in ('d', 'D'):
                        self.vy = -0.20
                    elif ch in ('q', 'Q'):
                        self.wz = 0.75
                    elif ch in ('e', 'E'):
                        self.wz = -0.75
                    elif ch == ' ':
                        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
                    elif ch in ('m', 'M'):
                        print("\n  💾 [KeyboardTeleop]: Map Save Triggered via 'M' key!\n")
                        if self.save_map_callback:
                            self.save_map_callback()
                    elif ch in ('x', 'X', '\x1b'):  # X or Esc
                        print("\n  🛑 [KeyboardTeleop]: Exiting Teleoperation.\n")
                        self.running = False
                        break

                    if self.command_callback:
                        self.command_callback(self.vx, self.vy, self.wz)
                    if ch in ('w', 'W', 's', 'S', 'a', 'A', 'd', 'D', 'q', 'Q', 'e', 'E', ' '):
                        print(f"  🎮 [Teleop]: vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}")

        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _run_watchdog(self):
        """Refinement 3: 0.40s Watchdog decays velocities to zero if keys released."""
        while self.running:
            time.sleep(0.05)
            elapsed = time.time() - self.last_cmd_time
            if elapsed > self.watchdog_timeout:
                if self.vx != 0.0 or self.vy != 0.0 or self.wz != 0.0:
                    self.vx = 0.0
                    self.vy = 0.0
                    self.wz = 0.0
                    if self.command_callback:
                        self.command_callback(0.0, 0.0, 0.0)

    def inject_command(self, vx: float, vy: float, wz: float):
        """Allows web HTTP API or external caller to inject a teleop pulse."""
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.last_cmd_time = time.time()
        if self.command_callback:
            self.command_callback(vx, vy, wz)
