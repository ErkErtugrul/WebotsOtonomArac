from vehicle import Driver
from controller import GPS, Compass, Camera
import math
import cv2
import numpy as np


# ============================================
# Gerçek hayat simülasyonu için 3 katmanlı mimari:
# 1. Global Path Planning (Waypoint sistemi - Google Maps benzeri)
# 2. Local Planning (Kamera ile engel tespiti)
# 3. Control Layer (Direksiyon/Gaz/Fren kontrolü)
# ============================================

class PathPlanner:
    """Google Maps'in yaptığını simüle eder"""
    
    def __init__(self):
        # waypointler
        # Gerçek hayatta burası Google Maps API'den gelecek
        self.waypoints = [
            (-37.0, -29.0),   # 1. Durak
            (45.0, -45.0),   # 2. Durak
            (28.0, -100.0),   # 3. Durak
            (-96.0, -96.0)    # Hedef
        ]
        """
        # deneme
        self.waypoints = [
            (current_x + 5, current_y + 5),   
            (current_x + 10, current_y + 10), 
        ]
        """
        self.current_waypoint_index = 0
        
    def get_current_waypoint(self):
        # Şu anki hedef noktayı döndür
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def update_waypoint(self, current_x, current_y, threshold=5.0):
        # Waypoint'e yaklaştıysak bir sonrakine geç
        waypoint = self.get_current_waypoint()
        if waypoint:
            dist = math.sqrt((waypoint[0] - current_x)**2 + (waypoint[1] - current_y)**2)
            if dist < threshold:
                self.current_waypoint_index += 1
                print(f"✓ Waypoint {self.current_waypoint_index} tamamlandı!")
                return True
        return False
    
    def is_route_complete(self):
        return self.current_waypoint_index >= len(self.waypoints)


class VisionSystem:
    def __init__(self):
        self.obstacle_threshold = 500 
        self.scale = 3
        self.roi_y_start = 0.4  
        self.roi_y_end = 0.95 
        
    def detect_obstacles(self, camera):
        # Kamera görüntüsünde engel tespit et
        try:
            camera_data = camera.getImage()
            if not camera_data:
                return {"left": 0, "center": 0, "right": 0}, None
            
            # Görüntüyü işle
            raw_image = np.frombuffer(camera_data, np.uint8).reshape(
                (camera.getHeight(), camera.getWidth(), 4)
            )
            frame_bgr = raw_image[:, :, :3]
            
            # Kenar tespiti
            frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            frame_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
            frame_canny = cv2.Canny(frame_blur, 50, 150)
            
            height, width = frame_canny.shape
            y1 = int(height * self.roi_y_start)
            y2 = int(height * self.roi_y_end)
            
            # Görüntüyü 3 bölgeye ayır
            third = width // 3
            
            roi_left = frame_canny[y1:y2, 0:third]
            roi_center = frame_canny[y1:y2, third:2*third]
            roi_right = frame_canny[y1:y2, 2*third:width]
            
            # Her bölgedeki engel yoğunluğu
            obstacles = {
                "left": np.count_nonzero(roi_left),
                "center": np.count_nonzero(roi_center),
                "right": np.count_nonzero(roi_right)
            }
            
            # Debug görüntüsü
            debug_img = self._create_debug_view(frame_canny, obstacles, y1, y2, third)
            
            return obstacles, debug_img
            
        except Exception as e:
            print(f"Kamera hatası: {e}")
            return {"left": 0, "center": 0, "right": 0}, None
    
    def _create_debug_view(self, frame_canny, obstacles, y1, y2, third):
        # Debug için görselleştirme
        display = cv2.cvtColor(frame_canny, cv2.COLOR_GRAY2BGR)
        height, width = frame_canny.shape
        
        # Bölgeleri çiz
        cv2.line(display, (third, 0), (third, height), (255, 0, 0), 2)
        cv2.line(display, (2*third, 0), (2*third, height), (255, 0, 0), 2)
        cv2.rectangle(display, (0, y1), (width, y2), (0, 255, 0), 2)
        
        # Engel yoğunluğunu göster
        cv2.putText(display, f"L:{obstacles['left']}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"C:{obstacles['center']}", (third+10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"R:{obstacles['right']}", (2*third+10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return cv2.resize(display, (width*self.scale, height*self.scale), 
                         interpolation=cv2.INTER_NEAREST)


class LocalPlanner:
    # Yol planlayıcı
    
    def __init__(self):
        self.max_speed = 40.0
        self.obstacle_threshold = 40
        self.emergency_distance = 0.5
        self.avoidance_distance = 2.0
        self.stuck_counter = 0
        self.max_stuck_time = 50
        
        self.state = "NORMAL"  # NORMAL, AVOIDING_LEFT, AVOIDING_RIGHT, EMERGENCY
        
    def plan_motion(self, obstacles, distance_sensor, target_angle, angle_diff):
        sensor_dist = distance_sensor.getValue() if distance_sensor else 10.0
        print(f" | Mesafe Sens: {sensor_dist:.2f}m", end="")
        # 1. ACİL FREN
        if sensor_dist < self.emergency_distance:
            self.state = "EMERGENCY"
            return {
                "speed": 0,
                "steering": 0,
                "brake": 1.0,
                "message": "⛔ ACİL FREN!"
            }
        
        # 2. ENGEL TESPİTİ
        left_clear = obstacles["left"] < self.obstacle_threshold
        center_clear = obstacles["center"] < self.obstacle_threshold
        right_clear = obstacles["right"] < self.obstacle_threshold
        
        # 3. ÖNDE ENGEL VAR MI?
        if not center_clear:
            
            # 3a. Uzak engel - Yavaşla ve karar ver
            if sensor_dist > self.avoidance_distance:
                self.state = "SLOWING"
                return {
                    "speed": self.max_speed * 0.3,
                    "steering": max(-0.5, min(0.5, angle_diff * 0.5)),
                    "brake": 0.0,
                    "message": "YAVAŞLIYOR (engel tespit)"
                }
            
            # 3b. Yakın engel - Manevra yap
            else:
                # Hangi taraf daha açık?
                if left_clear and right_clear:
                    # İkisi de açık - Hedefe göre seç
                    if angle_diff < 0:  # Hedef solda
                        chosen_side = "left"
                    else:  # Hedef sağda
                        chosen_side = "right"
                elif left_clear:
                    chosen_side = "left"
                elif right_clear:
                    chosen_side = "right"
                else:
                    # Her yer kapalı - DUR VE GERİ GİT
                    self.state = "BACKING"
                    return {
                        "speed": -15.0,
                        "steering": 0.5 if obstacles["left"] < obstacles["right"] else -0.5,
                        "brake": 0.0,
                        "message": "GERİ GİDİYOR"
                    }
                
                # Seçilen tarafa manevra
                if chosen_side == "left":
                    self.state = "AVOIDING_LEFT"
                    return {
                        "speed": self.max_speed * 0.4,
                        "steering": -0.6,  # Keskin sol
                        "brake": 0.0,
                        "message": "<- SOLA MANEVRA"
                    }
                else:
                    self.state = "AVOIDING_RIGHT"
                    return {
                        "speed": self.max_speed * 0.4,
                        "steering": 0.6,   # Keskin sağ
                        "brake": 0.0,
                        "message": "-> SAĞA MANEVRA"
                    }
        
        # 4. YOL AÇIK - Normal seyir
        self.state = "NORMAL"
        
        
        Kp = 0.8
        steering = Kp * angle_diff
        steering = max(-0.5, min(0.5, steering))
        
        # Hız kontrolü - Açıya göre ayarla
        angle_abs = abs(angle_diff)
        if angle_abs > 0.5:
            speed = self.max_speed * 0.4
        elif angle_abs > 0.3:
            speed = self.max_speed * 0.6
        else:
            speed = self.max_speed
        
        return {
            "speed": speed,
            "steering": steering,
            "brake": 0.0,
            "message": f"Normal seyir (açı: {math.degrees(angle_diff):.1f}°)"
        }


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def run():
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())
    
    gps = driver.getDevice("gps")
    compass = driver.getDevice("compass")
    camera = driver.getDevice("camera")
    dist_sensor = driver.getDevice("ds0")
    
    if gps:
        gps.enable(timestep)
    if compass:
        compass.enable(timestep)
    if camera:
        camera.enable(timestep)
    if dist_sensor:
        dist_sensor.enable(timestep)
    
            
    path_planner = PathPlanner()
    vision_system = VisionSystem()
    local_planner = LocalPlanner()
    
    print("=" * 60)
    print("OTONOM ARAÇ SİSTEMİ BAŞLATILDI")
    print("=" * 60)
    print(f"Rota: {len(path_planner.waypoints)} waypoint")
    print("Sensörler başlatılıyor...")
    
    # Sensörlerin hazır olması için bekle
    for _ in range(20):
        driver.step()
    
    print("Sistem hazır!\n")
    
    # ANA DÖNGÜ
    while driver.step() != -1:
        
        # 1. GLOBAL PLANLAMA
        current_waypoint = path_planner.get_current_waypoint()
        
        if current_waypoint is None:
            print("\nROTA TAMAMLANDI!")
            driver.setCruisingSpeed(0)
            driver.setBrakeIntensity(1.0)
            cv2.destroyAllWindows()
            break
        
        # 2. KONUM BİLGİSİ
        if gps and compass:
            gps_vals = gps.getValues()
            north = compass.getValues()
            
            current_x = gps_vals[0]
            current_y = gps_vals[1]
            
            
            # Waypoint'e ulaşıldı mı kontrol et
            path_planner.update_waypoint(current_x, current_y)
            current_waypoint = path_planner.get_current_waypoint()
            
            if current_waypoint is None:
                continue
            
            # Hedefe olan açıyı hesapla
            target_angle = math.atan2(
                current_waypoint[1] - current_y,
                current_waypoint[0] - current_x
            )
            current_angle = math.atan2(north[0], north[1])
            angle_diff = normalize_angle(current_angle - target_angle)
            #print(f"angle:{current_angle} , diff:{angle_diff}")
            
                  
            distance_to_waypoint = math.sqrt(
                (current_waypoint[0] - current_x)**2 + 
                (current_waypoint[1] - current_y)**2
            )
            
            
        # 3. GÖRÜŞ SİSTEMİ - Engel tespiti
        obstacles, debug_img = vision_system.detect_obstacles(camera)
        
        if debug_img is not None:
            cv2.imshow("Otonom Araç Görüşü", debug_img)
            cv2.waitKey(1)
        
        # 4. LOKAL PLANLAMA - Hareket kararı
        motion_plan = local_planner.plan_motion(
            obstacles, 
            dist_sensor,
            target_angle,
            angle_diff
        )
        
        # 5. KONTROL - Araç komutları
        driver.setCruisingSpeed(motion_plan["speed"])
        driver.setSteeringAngle(motion_plan["steering"])
        driver.setBrakeIntensity(motion_plan["brake"])
        
        t_deg = math.degrees(target_angle)
        c_deg = math.degrees(current_angle)
        diff_deg = math.degrees(angle_diff)
        
        
        # 6. DURUM BİLGİSİ

        print(f"\r{motion_plan['message']} | "
              f"Waypoint: {path_planner.current_waypoint_index + 1}/{len(path_planner.waypoints)} | "
              f"Mesafe: {distance_to_waypoint:.1f}m | "
              f"Engel: L:{obstacles['left']} C:{obstacles['center']} R:{obstacles['right']}", 
              end="")
        print(f"\t| Hedef: {t_deg:.1f}° | Yön: {c_deg:.1f}° | Fark: {diff_deg:.1f}°")
     
if __name__ == "__main__":
    run()