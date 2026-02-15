import requests
import threading

# SUNUCU AYARLARI 
SERVER_URL = "http://127.0.0.1:5000"

TARGET_POLL_EVERY    = 50    # her N adımda waypoint sorgula
TELEMETRY_SEND_EVERY = 30    # her N adımda telemetri gönder
COMMAND_POLL_EVERY   = 30    # her N adımda komut sorgula

class ServerComm:
    def __init__(self, url=SERVER_URL):
        self.url = url
        print(f"[Server] URL: {self.url}")

    # Waypoint sorgulama
    def poll_target(self, counter):
        if counter % TARGET_POLL_EVERY != 0:
            return None
        try:
            r    = requests.get(self.url + "/get_target", timeout=0.1)
            data = r.json()
            if data.get("lat") is not None and data.get("updated") is True:
                lat, lng = data["lat"], data["lng"]
                print(f"[Server] ▶ Yeni hedef → {lat:.6f}, {lng:.6f}")
                return lat, lng
        except Exception:
            pass
        return None

    # Komut sorgulama (YENİ)
    def poll_command(self, counter):
        if counter % COMMAND_POLL_EVERY != 0:
            return None
        try:
            r    = requests.get(self.url + "/get_command", timeout=0.1)
            data = r.json()
            if data.get("updated") is True:
                action = data.get("action", "RUN")
                print(f"[Server] Komut → {action}")
                return action
        except Exception:
            pass
        return None

    # Telemetri
    def send_telemetry(self, counter, lat, lng, heading,
                       speed, dist, nav_state, cam_obstacles):
        if counter % TELEMETRY_SEND_EVERY != 0:
            return
        self._post_async("/vehicle_update", {
            "lat":            lat,
            "lng":            lng,
            "heading":        heading,
            "speed":          speed,
            "dist_to_target": dist,
            "obstacle_mode":  nav_state,
            "cam_obstacles":  cam_obstacles
        })

    # Waypoint bildirimi
    def notify_waypoint_reached(self, lat, lng):
        # Waypoint'e ulaşıldığında sunucuya bildirir.
        self._post_async("/waypoint_reached", {
            "waypoint": {"lat": lat, "lng": lng}
        })
        print(f"[Server] Waypoint bildirildi: {lat:.6f}, {lng:.6f}")

    # Yardımcı 
    def _post_async(self, endpoint, payload):
        def _go():
            try:
                requests.post(self.url + endpoint, json=payload, timeout=0.3)
            except Exception:
                pass
        threading.Thread(target=_go, daemon=True).start()