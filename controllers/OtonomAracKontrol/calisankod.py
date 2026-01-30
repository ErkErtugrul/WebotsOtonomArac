from vehicle import Driver
from controller import GPS, Compass, Camera
import math
import cv2
import numpy as np

class PathPlanner:
    def __init__(self):
        self.waypoints = [
            (-37.0, -29.0),
            (45.0, -45.0),
            (28.0, -100.0),
            (-96.0, -96.0)
        ]
        self.current_waypoint_index = 0
        
    def get_current_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
        
    def update_waypoint(self, current_x, current_y, threshold=5.0):
        waypoint = self.get_current_waypoint()
        if waypoint:
            dist = math.sqrt((waypoint[0] - current_x)**2 + (waypoint[1] - current_y)**2)
            if dist < threshold:
                self.current_waypoint_index += 1
                return True
        return False
        
    def is_route_complete(self):
        return self.current_waypoint_index >= len(self.waypoints)

class VisionSystem:
    def __init__(self):
        self.obstacle_threshold = 50
        self.scale = 3
        self.roi_y_start = 0.5
        self.roi_y_end = 0.9
        
    def detect_obstacles(self, camera):
        try:
            camera_data = camera.getImage()
            if not camera_data:
                return {"left": 0, "center": 0, "right": 0}, None
            
            raw_image = np.frombuffer(camera_data, np.uint8).reshape(
                (camera.getHeight(), camera.getWidth(), 4)
            )
            frame_bgr = raw_image[:, :, :3]
            
            frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            frame_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
            frame_canny = cv2.Canny(frame_blur, 50, 150)
            
            height, width = frame_canny.shape
            y1 = int(height * self.roi_y_start)
            y2 = int(height * self.roi_y_end)
            
            third = width // 3
            
            roi_left = frame_canny[y1:y2, 0:third]
            roi_center = frame_canny[y1:y2, third:2*third]
            roi_right = frame_canny[y1:y2, 2*third:width]
            
            obstacles = {
                "left": np.count_nonzero(roi_left),
                "center": np.count_nonzero(roi_center),
                "right": np.count_nonzero(roi_right)
            }
            
            debug_img = self._create_debug_view(frame_canny, obstacles, y1, y2, third)
            return obstacles, debug_img
            
        except Exception:
            return {"left": 0, "center": 0, "right": 0}, None
    
    def _create_debug_view(self, frame_canny, obstacles, y1, y2, third):
        display = cv2.cvtColor(frame_canny, cv2.COLOR_GRAY2BGR)
        height, width = frame_canny.shape
        
        cv2.line(display, (third, 0), (third, height), (255, 0, 0), 2)
        cv2.line(display, (2*third, 0), (2*third, height), (255, 0, 0), 2)
        cv2.rectangle(display, (0, y1), (width, y2), (0, 255, 0), 2)
        
        cv2.putText(display, f"L:{obstacles['left']}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"C:{obstacles['center']}", (third+10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display, f"R:{obstacles['right']}", (2*third+10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return cv2.resize(display, (width*self.scale, height*self.scale), 
                          interpolation=cv2.INTER_NEAREST)

class LocalPlanner:
    def __init__(self):
        self.max_speed = 40.0
        self.obstacle_threshold = 40
        self.emergency_distance = 0.5
        self.avoidance_distance = 2.0
        self.state = "NORMAL"
        
    def plan_motion(self, obstacles, distance_sensor, target_angle, angle_diff):
        sensor_dist = distance_sensor.getValue() if distance_sensor else 10.0
        
        if sensor_dist < self.emergency_distance:
            self.state = "EMERGENCY"
            return {"speed": 0, "steering": 0, "brake": 1.0, "message": "ACIL FREN"}
        
        left_clear = obstacles["left"] < self.obstacle_threshold
        center_clear = obstacles["center"] < self.obstacle_threshold
        right_clear = obstacles["right"] < self.obstacle_threshold
        
        if not center_clear and sensor_dist < self.avoidance_distance:
            if left_clear and obstacles["left"] < obstacles["right"]:
                self.state = "AVOIDING_LEFT"
                return {"speed": self.max_speed * 0.4, "steering": -0.5, "brake": 0.0, "message": "SOLA KACIYOR"}
            elif right_clear:
                self.state = "AVOIDING_RIGHT"
                return {"speed": self.max_speed * 0.4, "steering": 0.5, "brake": 0.0, "message": "SAGA KACIYOR"}
            else:
                return {"speed": -10.0, "steering": 0.3, "brake": 0.0, "message": "GERI GIDIYOR"}
        
        self.state = "NORMAL"
        Kp = 0.6   # 0.4–0.8 arası ideal
        steering = Kp * angle_diff
        steering = max(-0.5, min(0.5, steering))
                
        angle_abs = abs(angle_diff)
        speed = self.max_speed * max(0.3, 1 - angle_abs)
        """
        if abs(angle_diff) > 0.3:
            speed = self.max_speed * 0.5
        elif abs(angle_diff) > 0.15:
            speed = self.max_speed * 0.7
        else:
            speed = self.max_speed
            """
        return {"speed": speed, "steering": steering, "brake": 0.0, "message": f"Hedef aci: {math.degrees(angle_diff):.1f}"}

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
    
    if gps: gps.enable(timestep)
    if compass: compass.enable(timestep)
    if camera: camera.enable(timestep)
    if dist_sensor: dist_sensor.enable(timestep)
            
    path_planner = PathPlanner()
    vision_system = VisionSystem()
    local_planner = LocalPlanner()
    
    for _ in range(20):
        driver.step()
    
    while driver.step() != -1:
        current_waypoint = path_planner.get_current_waypoint()
        
        if current_waypoint is None:
            driver.setCruisingSpeed(0)
            driver.setBrakeIntensity(1.0)
            cv2.destroyAllWindows()
            break
        
        if gps and compass:
            gps_vals = gps.getValues()
            north = compass.getValues()
            
            current_x = gps_vals[0]
            current_y = gps_vals[1]
            
            path_planner.update_waypoint(current_x, current_y)
            current_waypoint = path_planner.get_current_waypoint()
            
            if current_waypoint is None:
                continue
            
            target_angle = math.atan2(
                current_waypoint[1] - current_y,
                current_waypoint[0] - current_x
            )
            
            current_angle = math.atan2(north[0], north[1])
            angle_diff = normalize_angle(current_angle - target_angle)
            
            print(f"\nCompass: X={north[0]:.3f}, Z={north[2]:.3f}")
            print(f"Current angle: {math.degrees(current_angle):.1f}°")
            print(f"Target angle: {math.degrees(target_angle):.1f}°")
            print(f"Angle diff: {math.degrees(angle_diff):.1f}°")
            print(f"Position: ({current_x:.1f}, {current_y:.1f})")
            print(f"Target: {current_waypoint}")
        else:
            angle_diff = 0
            target_angle = 0
            
        obstacles, debug_img = vision_system.detect_obstacles(camera)
        
        if debug_img is not None:
            cv2.imshow("Vision", debug_img)
            cv2.waitKey(1)
        
        motion_plan = local_planner.plan_motion(
            obstacles, 
            dist_sensor,
            target_angle,
            angle_diff
        )
        
        driver.setCruisingSpeed(motion_plan["speed"])
        driver.setSteeringAngle(motion_plan["steering"])
        driver.setBrakeIntensity(motion_plan["brake"])
        
        print(f"\r{motion_plan['message']}", end="")

if __name__ == "__main__":
    run()