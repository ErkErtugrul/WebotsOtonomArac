from vehicle import Driver
from controller import GPS, Compass, Camera
import math
import cv2
import numpy as np

TARGET_X = -96.0   
TARGET_Y = -96.0  
MAX_SPEED = 40.0 
STOP_DIST = 2.0 

# GÖRÜNTÜ BÜYÜTME
SCALE = 5 

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

    print("Sensörler başlatılıyor")
    for _ in range(20): 
        driver.step()

    print(f"Hedef: X={TARGET_X}, Y={TARGET_Y}")

    while driver.step() != -1:
        
        # GÖRÜNTÜ İŞLEME
        if cam:
            try:
                camera_data = cam.getImage()
                
                if camera_data:
                    raw_image = np.frombuffer(camera_data, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
                    frame_bgr = raw_image[:, :, :3] # Renkli

                    frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # Gri
                    frame_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
                    frame_canny = cv2.Canny(frame_blur, 50, 150) # Kenar

                    height, width = frame_bgr.shape[:2]
                    new_dim = (width * SCALE, height * SCALE)

                    display_rgb = cv2.resize(frame_bgr, new_dim, interpolation=cv2.INTER_NEAREST)
                    display_gray = cv2.resize(frame_gray, new_dim, interpolation=cv2.INTER_NEAREST)
                    display_canny = cv2.resize(frame_canny, new_dim, interpolation=cv2.INTER_NEAREST)

                    cv2.imshow("1", display_rgb)
                    cv2.imshow("gri", display_gray)
                    cv2.imshow("kenarlar", display_canny)
                    
                    cv2.waitKey(1)

            except Exception as e:
                pass

        if gps and compass:
            vals = gps.getValues()
            north = compass.getValues()
            
            cx = vals[0]
            cy = vals[1] 
            
            dist_to_target = math.sqrt((TARGET_X - cx)**2 + (TARGET_Y - cy)**2)
            
            if dist_to_target < STOP_DIST:
                driver.setCruisingSpeed(0)
                driver.setBrakeIntensity(1.0)
                print("Hedefe Ulaşıldı!")
                cv2.destroyAllWindows()
                break

            target_angle = math.atan2(TARGET_X - cx, TARGET_Y - cy)
            current_angle = math.atan2(north[1], north[0])
            angle_diff = normalize_angle(target_angle - current_angle)

            steering_command = angle_diff
            steering_command = max(-0.5, min(0.5, steering_command))
            
            driver.setSteeringAngle(steering_command)
            
            if abs(angle_diff) > 0.2:
                driver.setCruisingSpeed(MAX_SPEED * 0.3)
            else:
                driver.setCruisingSpeed(MAX_SPEED)

if __name__ == "__main__":
    run()