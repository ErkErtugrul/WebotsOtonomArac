from vehicle import Driver
from controller import GPS, Compass, Camera
import math
import cv2
import numpy as np

# AYARLAR
TARGET_X = -96.0
TARGET_Y = -96.0
MAX_SPEED = 35.0
OBSTACLE_SPEED = 15.0
STOP_DIST = 2.0

# HSV
LOWER_LANE = np.array([100, 10, 43])
UPPER_LANE = np.array([123, 23, 49])

STATE_DRIVE = "DRIVING"       # Normal sürüş
STATE_AVOID = "AVOIDING"      # Engelden kaçma 
STATE_RECOVER = "RECOVERING"  # Engelden kurtuldu, yola devam

class AutonomousCar:
    def __init__(self):
        self.driver = Driver()
        self.timestep = int(self.driver.getBasicTimeStep())
        
        self.gps = self.driver.getDevice("gps")
        self.compass = self.driver.getDevice("compass")
        self.cam = self.driver.getDevice("camera")
        

        if self.gps: self.gps.enable(self.timestep)
        if self.compass: self.compass.enable(self.timestep)
        if self.cam: self.cam.enable(self.timestep)

        
        self.current_state = STATE_DRIVE
        self.avoid_counter = 0        
        self.obstacle_last_seen = 0   

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def get_camera_data(self):
        """Kameradan hem şerit hem engel verisini çeker."""
        data = self.cam.getImage()
        if not data: return None, False, 0, False, 0
        
        img_bgra = np.frombuffer(data, np.uint8).reshape((self.cam.getHeight(), self.cam.getWidth(), 4))
        image = cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
        h, w, _ = image.shape

        # 1. Engel Tespiti
        
        obs_roi = image[int(h*0.4):int(h*0.8), int(w*0.3):int(w*0.7)]
        gray = cv2.cvtColor(obs_roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        obs_detected = False
        obs_center_x = 0
        
        edge_score = np.sum(edges) / 255
        if edge_score > 1000:
            obs_detected = True
            M = cv2.moments(edges)
            if M["m00"] > 0:
                local_cx = int(M["m10"] / M["m00"])
                obs_center_x = local_cx + int(w*0.3) 
                cv2.rectangle(image, (int(w*0.3), int(h*0.4)), (int(w*0.7), int(h*0.8)), (0,0,255), 2)

        # 2. Şerit Tespiti
        lane_roi = image[int(h*0.65):h, 0:w]
        hsv = cv2.cvtColor(lane_roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_LANE, UPPER_LANE)
        
        lane_found = False
        lane_error = 0
        M_lane = cv2.moments(mask)
        if M_lane["m00"] > 0:
            cx = int(M_lane["m10"] / M_lane["m00"])
            lane_error = cx - (w // 2)
            lane_found = True
            cv2.circle(image, (cx, int(h*0.8)), 5, (0, 255, 0), -1)

        cv2.imshow("Vision", image)
        cv2.waitKey(1)
        
        return image, obs_detected, obs_center_x, lane_found, lane_error

    def run(self):
        print("Sistem Başlatıldı. Sürüş Modu: OTONOM")
        
        while self.driver.step() != -1:
            image, obs_detected, obs_x, lane_found, lane_error = self.get_camera_data()
            
            # GPS Verileri
            vals = self.gps.getValues()
            north = self.compass.getValues()
            target_angle = math.atan2(TARGET_X - vals[0], TARGET_Y - vals[1])
            heading = math.atan2(north[1], north[0])
            gps_steering = self.normalize_angle(target_angle - heading)
            dist_to_target = math.sqrt((TARGET_X - vals[0])**2 + (TARGET_Y - vals[1])**2)
    
            steering = 0.0
            speed = MAX_SPEED
            force_avoid = 0.0
        
            left_val = ds_left.getValue()
            right_val = ds_right.getValue()
            
            if left_val > 80:
                force_avoid += (left_val / 1000.0) 
                
            if right_val > 80:
                force_avoid -= (right_val / 1000.0)

            # 3. Path Planning
            total_steering = (force_target * TARGET_WEIGHT) - (force_avoid * AVOID_WEIGHT)
            
         
            total_steering = max(-0.5, min(0.5, total_steering))
              

            # hdefe vardık mı?
            if dist_to_target < STOP_DIST:
                print("Hedefe Ulaşıldı!")
                self.driver.setCruisingSpeed(0)
                self.driver.setBrakeIntensity(1.0)
                break

            # Karar
            
            if self.current_state == STATE_DRIVE and obs_detected:
                print("ENGEL!!!!")
                self.current_state = STATE_AVOID
                self.avoid_counter = 100 
                
                h, w, _ = image.shape
                if obs_x > w/2:
                    self.avoid_direction = -0.4 
                else:
                    self.avoid_direction = 0.4  

            
            if self.current_state == STATE_AVOID:
                steering = self.avoid_direction
                speed = OBSTACLE_SPEED
                self.avoid_counter -= 1

                if self.avoid_counter <= 0:
                    self.current_state = STATE_RECOVER
                    self.avoid_counter = 50 

            elif self.current_state == STATE_RECOVER:
                print("Yola Dönülüyor...")
                steering = gps_steering 
                speed = OBSTACLE_SPEED
                self.avoid_counter -= 1
                
                if lane_found:
                    self.current_state = STATE_DRIVE
                elif self.avoid_counter <= 0:
                    self.current_state = STATE_DRIVE

            elif self.current_state == STATE_DRIVE:
                if lane_found:
                    steering = (lane_error * 0.005) + (gps_steering * 0.1)
                else:
                    steering = gps_steering

            steering = max(-0.5, min(0.5, steering))
            
            self.driver.setSteeringAngle(steering)
            self.driver.setCruisingSpeed(speed)

if __name__ == "__main__":
    bot = AutonomousCar()
    bot.run()