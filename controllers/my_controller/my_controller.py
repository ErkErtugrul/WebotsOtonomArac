from vehicle import Driver
from controller import GPS, Compass, Camera
import math
import cv2
import numpy as np

# Gideceği konum
TARGET_X = -96.0   
TARGET_Y = -96.0  
MAX_SPEED = 40.0 
STOP_DIST = 2.0 

# GÖRÜNTÜ BÜYÜTME
SCALE = 5 

OBSTACLE_THRESHOLD = 40 # CANNY çizgileri limiti
ROI_START_Y = 0.5   # Görüntünün ortasından başla
ROI_END_Y = 0.9     # Biraz aşağıya kadar bak
ROI_START_X = 0.4   # Soldan %30 içerisi
ROI_END_X = 0.6     # Sağdan %30 içerisi

AVOIDANCE_DIST = 1.5   # Engel en uzak
EMERGENCY_DIST = 0.3   # Engel en yakın

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
    cam = driver.getDevice("camera") 

    if gps:
        gps.enable(timestep)
    if compass:
        compass.enable(timestep)
    if cam:
        cam.enable(timestep)
        
    dist_sensor = driver.getDevice("ds0") 
    if dist_sensor:
        dist_sensor.enable(timestep)
        
    print("Sensörler başlatılıyor")
    # Sensörler için küçük bir bekleme
    for _ in range(20): 
        driver.step()

    print(f"Hedef: X={TARGET_X}, Y={TARGET_Y}")

    while driver.step() != -1:
        
        # GÖRÜNTÜ İŞLEME
        obstacle_detected = False
        if cam:
            try:
                camera_data = cam.getImage()
                
                if camera_data:
                    raw_image = np.frombuffer(camera_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
                    frame_bgr = raw_image[:, :, :3] # Renkli

                    frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # Gri
                    frame_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
                    frame_canny = cv2.Canny(frame_blur, 50, 150) # Kenar

                    height, width = frame_canny.shape
                    
                    # ROI Koordinatlarını Hesapla
                    y1 = int(height * ROI_START_Y)
                    y2 = int(height * ROI_END_Y)
                    x1 = int(width * ROI_START_X)
                    x2 = int(width * ROI_END_X)
                    
                    # Sadece bu alana odaklan (Kesip alıyoruz)
                    roi = frame_canny[y1:y2, x1:x2]
                    
                    white_pixels = np.count_nonzero(roi)
                    print(f"Algılanan Piksel Sayısı: {white_pixels} / Eşik: {OBSTACLE_THRESHOLD}")
                    if white_pixels > OBSTACLE_THRESHOLD:
                        obstacle_detected = True
                        print("ENGEL TESPİT EDİLDİ! FREN YAPILIYOR.")
                        
                    new_dim = (width * SCALE, height * SCALE)
                    
                    display_debug = cv2.cvtColor(frame_canny, cv2.COLOR_GRAY2BGR)
                    color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
                    cv2.rectangle(display_debug, (x1, y1), (x2, y2), color, 2)
                    cv2.imshow("Ana Goruntu (Kutulu)", cv2.resize(display_debug, (width*3, height*3), interpolation=cv2.INTER_NEAREST))
                    display_canny = cv2.resize(frame_canny, new_dim, interpolation=cv2.INTER_NEAREST)
                    display_roi = cv2.resize(frame_canny, new_dim, interpolation=cv2.INTER_NEAREST)
                    
                    roi_scale = 10  
                    roi_h, roi_w = roi.shape
                    display_roi = cv2.resize(roi, (roi_w * roi_scale, roi_h * roi_scale), interpolation=cv2.INTER_NEAREST)
                    
                    roi_left = frame_canny[y1:y2, x1:int((x1+x2)/2)]
                    roi_right = frame_canny[y1:y2, int((x1+x2)/2):x2]
                    
                    left_pixels = np.count_nonzero(roi_left)
                    right_pixels = np.count_nonzero(roi_right)
                    
                    if left_pixels < right_pixels:
                        driver.setSteeringAngle(-0.4)  # Sola
                    else:
                        driver.setSteeringAngle(0.4)
                        
                    cv2.imshow("kenarlar", display_canny)
                    cv2.imshow("Sadece ROI (Göz)", display_roi)
                    cv2.waitKey(1)

            except Exception as e:
                print("Kamera hatası:", e)
        
        
        if gps and compass:
            vals = gps.getValues()
            north = compass.getValues()
            
            cx = vals[0]
            cy = vals[2] # 2 olursa z eksenini alınır
            
            dist_to_target = math.sqrt((TARGET_X - cx)**2 + (TARGET_Y - cy)**2)
            
            if dist_to_target < STOP_DIST:
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1.0)
                print("Hedefe Ulaşıldı!")
                cv2.destroyAllWindows()
                break

            target_angle = math.atan2(TARGET_X - cx, TARGET_Y - cy)
            current_angle = math.atan2(north[0], north[2]) # z ekseni alınıcaksa north[0], north[2]
            angle_diff = normalize_angle(target_angle - current_angle)

            steering_command = angle_diff
            steering_command = max(-0.5, min(0.5, steering_command))
                        
            actual_distance = 10.0
            if dist_sensor:
                actual_distance = dist_sensor.getValue()
            
            print(f"Kamera: {obstacle_detected} | Mesafe: {actual_distance:.2f}")
            
            if actual_distance < EMERGENCY_DIST:
                print("!!! ACİL FREN - ÇARPIŞMA RİSKİ !!!")
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1.0)

            elif obstacle_detected and actual_distance < AVOIDANCE_DIST:
                print("--- Engelden Kaçılıyor (Sola Kırılıyor) ---")
                driver.setCruisingSpeed(MAX_SPEED * 0.5)
                driver.setBrakeIntensity(0.0)
                driver.setSteeringAngle(-0.4) 

            else:
                driver.setBrakeIntensity(0.0)
                driver.setSteeringAngle(steering_command)
                
                if abs(angle_diff) > 0.2:
                     driver.setCruisingSpeed(MAX_SPEED * 0.3)
                else:
                     driver.setCruisingSpeed(MAX_SPEED)

if __name__ == "__main__":
    run()