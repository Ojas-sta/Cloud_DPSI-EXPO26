"""
Project Cloud: Incident Webhook Dispatcher
Dispatches rich hazard alerts and photo evidence to Discord/Slack webhooks.
Reference: PROJECT.md Section 7.5 & 8.4
"""

import os
import json
import urllib.request
import urllib.error
from typing import Optional, Dict, Any
from datetime import datetime

from cloud_core.bus import CloudNode
from cloud_core.types import SpillAlert

class DiscordDispatcherNode(CloudNode):
    """
    Subscribes to /cloud/spill_hazard and posts structured incident embeds.
    """
    def __init__(self, node_name: str = "discord_dispatcher", webhook_url: Optional[str] = None):
        super().__init__(node_name)
        self.webhook_url = webhook_url or os.environ.get("DISCORD_WEBHOOK_URL", "")
        self.last_dispatched_payload: Optional[Dict[str, Any]] = None

        # Subscriptions
        self.sub_spill = self.create_subscription(
            SpillAlert, "/cloud/spill_hazard", self.dispatch_alert
        )

    def dispatch_alert(self, alert: SpillAlert) -> Dict[str, Any]:
        """Formats and posts the incident embed."""
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        embed = {
            "title": "🚨 CRITICAL HAZARD: LIQUID SPILL DETECTED",
            "description": "Autonomous stewardship agent **Project Cloud (ARGUS)** has intercepted an active floor spill in the pavilion.",
            "color": 15158332,  # Crimson Red
            "fields": [
                {
                    "name": "Hazard Classification",
                    "value": f"`{alert.hazard_type}`",
                    "inline": True
                },
                {
                    "name": "Severity",
                    "value": f"**{alert.severity}**",
                    "inline": True
                },
                {
                    "name": "Detection Confidence",
                    "value": f"{alert.confidence:.1%}",
                    "inline": True
                },
                {
                    "name": "Estimated Spill Area",
                    "value": f"{alert.estimated_area_sq_m:.2f} m²",
                    "inline": True
                },
                {
                    "name": "Relative Position",
                    "value": f"X: {alert.location.x:.2f}m, Y: {alert.location.y:.2f}m",
                    "inline": True
                },
                {
                    "name": "Containment Protocol",
                    "value": "Quadruped frozen in warning stance. Visual strobe engaged.",
                    "inline": False
                }
            ],
            "footer": {
                "text": f"Project Cloud // DPSI EXPO 2026 // {now_str}"
            }
        }

        payload = {
            "username": "Cloud Safety Sentinel",
            "avatar_url": "https://ojas-sta.github.io/Cloud_DPSI-EXPO26/assets/cloud-avatar.png",
            "embeds": [embed]
        }

        self.last_dispatched_payload = payload
        self.logger.info(f"INCIDENT WEBHOOK DISPATCHED: Severity={alert.severity}, Area={alert.estimated_area_sq_m}m^2")

        if self.webhook_url and self.webhook_url.startswith("http"):
            try:
                data = json.dumps(payload).encode("utf-8")
                req = urllib.request.Request(
                    self.webhook_url,
                    data=data,
                    headers={"Content-Type": "application/json", "User-Agent": "ProjectCloud-SafetyAgent"}
                )
                with urllib.request.urlopen(req, timeout=3.0) as resp:
                    self.logger.info(f"Webhook response status: {resp.status}")
            except Exception as e:
                self.logger.warn(f"Could not reach external webhook: {e}")

        return payload
