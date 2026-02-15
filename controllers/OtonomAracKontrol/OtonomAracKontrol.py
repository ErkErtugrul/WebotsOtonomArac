from vehicle import Driver
from controller import Keyboard, Camera
import math
import cv2
import numpy as np

# ============================================
# TEST MODU: MANUEL KONTROL & ENGEL UYARISI
# ============================================

class VisionSystem:
    def __init__(self):
        # BU AYARLARLA OYNAYARAK GÖRÜNTÜ İŞLEMEYİ DÜZELTEBİLİRSİN
        self.obstacle_threshold = 100  # Engel sayılma eşiği (pixel sayısı)
        self.scale = 3                 # Ekranda gösterilen pencere boyutu çarpanı
        self.roi_y_start = 0.4         # İlgi alanı (Region of Interest) başlangıcı
        self.roi_y_end = 0.95          # İlgi alanı bitişi
        
        # Canny parametreleri
        self.canny_low = 50
        self.canny_high = 150
        
    def detect_obstacles(self, camera):
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
            frame_canny = cv2.Canny(frame_blur, self.canny_low, self.canny_high)
            
            height, width = frame_canny.shape
            y1 = int(height * self.roi_y_start)
            y2 = int(height * self.roi_y_end)
            third = width // 3
            
            # ROI bölgeleri
            roi_left = frame_canny[y1:y2, 0:third]
            roi_center = frame_canny[y1:y2, third:2*third]
            roi_right = frame_canny[y1:y2, 2*third:width]
            
            # Piksel sayımı
            obstacles = {
                "left": np.count_nonzero(roi_left),
                "center": np.count_nonzero(roi_center),
                "right": np.count_nonzero(roi_right)
            }
            
            debug_img = self._create_debug_view(frame_canny, obstacles, y1, y2, third)
            return obstacles, debug_img
            
        except Exception as e:
            print(f"Kamera hatası: {e}")
            return {"left": 0, "center": 0, "right": 0}, None
    
    def _create_debug_view(self, frame_canny, obstacles, y1, y2, third):
        display = cv2.cvtColor(frame_canny, cv2.COLOR_GRAY2BGR)
        height, width = frame_canny.shape
        
        # Çizgiler
        cv2.line(display, (third, 0), (third, height), (255, 0, 0), 1)
        cv2.line(display, (2*third, 0), (2*third, height), (255, 0, 0), 1)
        cv2.rectangle(display, (0, y1), (width, y2), (0, 255, 0), 2)
        
        # Görsel üzerine yazı (Debug ekranı için)
        color_l = (0, 0, 255) if obstacles['left'] > self.obstacle_threshold else (0, 255, 0)
        color_c = (0, 0, 255) if obstacles['center'] > self.obstacle_threshold else (0, 255, 0)
        color_r = (0, 0, 255) if obstacles['right'] > self.obstacle_threshold else (0, 255, 0)

        cv2.putText(display, f"L:{obstacles['left']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_l, 2)
        cv2.putText(display, f"C:{obstacles['center']}", (third+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_c, 2)
        cv2.putText(display, f"R:{obstacles['right']}", (2*third+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_r, 2)
        
        return cv2.resize(display, (width*self.scale, height*self.scale), interpolation=cv2.INTER_NEAREST)


def run():
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())
    
    keyboard = driver.getKeyboard()
    keyboard.enable(timestep)

    camera = driver.getDevice("camera")
    if camera:
        camera.enable(timestep)
        
    vision_system = VisionSystem()
    
    print("=" * 60)
    print("MANUEL DEBUG MODU")
    print("Kontroller: YÖN TUŞLARI")
    print("Odak: 3D Pencereye tıkla")
    print("=" * 60)
    
    MAX_SPEED = 50.0
    MAX_STEER = 0.5
    current_speed = 0
    current_steering = 0
    
    while driver.step() != -1:
        # 1. Manuel Kontrol
        key = keyboard.getKey()
        if key == Keyboard.UP: current_speed = MAX_SPEED
        elif key == Keyboard.DOWN: current_speed = -15.0
        else: current_speed = 0
            
        if key == Keyboard.LEFT: current_steering = -MAX_STEER
        elif key == Keyboard.RIGHT: current_steering = MAX_STEER
        else: current_steering = 0
            
        driver.setCruisingSpeed(current_speed)
        driver.setSteeringAngle(current_steering)
        driver.setBrakeIntensity(0)
        
        # 2. Görüş Sistemi
        obstacles, debug_img = vision_system.detect_obstacles(camera)
        
        if debug_img is not None:
            cv2.imshow("Vision Debug Monitor", debug_img)
            cv2.waitKey(1)
            
        # 3. KONSOL BİLGİLENDİRME (Burayı Düzenledim)
        thr = vision_system.obstacle_threshold
        
        # Hangi bölgelerde engel var?
        detected_zones = []
        if obstacles['left'] > thr: detected_zones.append("SOL")
        if obstacles['center'] > thr: detected_zones.append("ORTA")
        if obstacles['right'] > thr: detected_zones.append("SAĞ")
        
        # Mesajı oluştur
        if len(detected_zones) > 0:
            status_msg = f"⚠️  ENGEL VAR! [{' '.join(detected_zones)}]"
        else:
            status_msg = "✅ YOL TEMİZ"
            
        # Ekrana yazdır (Boşluklarla hizalama yapıyoruz ki görüntü titremesin)
       print(f"\rDeğerler: L:{VisionSystem().obstacles['left']:<4} C:{VisionSystem().obstacles['center']:<4} R:{VisionSystem().obstacles['right']:<4}", end="")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    run()