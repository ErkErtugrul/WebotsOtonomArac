from vehicle import Driver

from navigation import (
    PIDController,
    latlon_to_meters, meters_to_latlon, haversine_m,
    get_heading, clamp,
    compute_nav_errors, target_speed,
    avoid_direction, facing_target,
    MAX_SPEED_KMH, MAX_STEER_ANGLE,
    STEER_KP, STEER_KI, STEER_KD, STEER_I_MAX,
    SPEED_KP,  SPEED_KI,  SPEED_KD,  SPEED_I_MAX,
)
from vision  import VisionSystem
from planner import LocalPlanner, DS_CLEAR_DIST, CLEAR_STEPS_NEEDED, AVOID_SPEED_KMH, AVOID_STEER
from server  import ServerComm

driver   = Driver()
timestep = int(driver.getBasicTimeStep())
dt       = timestep / 1000.0

print("=" * 60)
print("  OTONOM ARAÇ — FinalController")
print("=" * 60)

# Cihaz listesi aracın cihazlarını öğrenmek için kullanılır
# print("=== Cihaz Listesi ===")
# for i in range(driver.getNumberOfDevices()):
#     print(f"  {i}: {driver.getDeviceByIndex(i).getName()}")
# print("=====================\n")

def init_device(name):
    d = driver.getDevice(name)
    if d:
        d.enable(timestep)
        print(f"{name} aktif")
    else:
        print(f"HATA: '{name}' bulunamadı!")
    return d

gps     = init_device("gps")
compass = init_device("compass")
camera  = init_device("camera")
ds      = init_device("ds0")

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)
driver.setBrakeIntensity(0.0)

print("\nSensörler ısınıyor...")
for _ in range(20):
    driver.step()
print("Sistem hazır!\n")

vision        = VisionSystem()
local_planner = LocalPlanner()
server        = ServerComm()

steer_pid = PIDController(STEER_KP, STEER_KI, STEER_KD,
                          -MAX_STEER_ANGLE, MAX_STEER_ANGLE, STEER_I_MAX)
speed_pid = PIDController(SPEED_KP, SPEED_KI, SPEED_KD,
                          -MAX_SPEED_KMH,   MAX_SPEED_KMH,   SPEED_I_MAX)

print(f"PID Direksiyon : Kp={STEER_KP}  Ki={STEER_KI}  Kd={STEER_KD}")
print(f"PID Hız        : Kp={SPEED_KP}  Ki={SPEED_KI}  Kd={SPEED_KD}\n")

hedef_x        = None
hedef_z        = None
hedef_lat      = None
hedef_lng      = None
hedefe_ulasti  = False
current_speed  = 0.0
nav_state        = 'NORMAL'
stored_avoid_dir = 0
clear_step_count = 0
counter          = 0
debug_counter    = 0

# her x adımda bir detaylı çıktı ver
DEBUG_EVERY = 60   # 0 = kapalı

print("Haritadan ilk waypoint bekleniyor...\n")


# ANA DÖNGÜ

while driver.step() != -1:
    counter       += 1
    debug_counter += 1

    # 1) SENSÖR
    gps_vals     = gps.getValues() if gps else [0.0, 0.0, 0.0]
    rx, rz       = gps_vals[0], gps_vals[2]
    heading      = get_heading(compass.getValues()) if compass else 0.0
    raw_ds       = ds.getValue() if ds else 0.0
    ds_dist      = raw_ds if raw_ds > 0.01 else 999.0
    robot_lat, robot_lng = meters_to_latlon(rx, rz)

    # 2) KAMERA
    if camera:
        obstacles, debug_img = vision.analyze(camera)
        vision.show(debug_img, nav_state, ds_dist, current_speed)
    else:
        obstacles = {'left': 0, 'center': 0, 'right': 0,
                     'left_blocked': False, 'center_blocked': False, 'right_blocked': False}

    # 3a) HEDEF
    new_target = server.poll_target(counter)
    if new_target:
        hedef_lat, hedef_lng = new_target
        hedef_x, hedef_z     = latlon_to_meters(hedef_lat, hedef_lng)
        hedefe_ulasti        = False
        nav_state            = 'NORMAL'
        clear_step_count     = 0
        steer_pid.reset()
        speed_pid.reset()
        current_speed = 0.0
        print(f"Yeni hedef  → Webots: X={hedef_x:.2f}m  Z={hedef_z:.2f}m")
        print(f"Araç konumu → rx={rx:.2f}m  rz={rz:.2f}m")
       
        # Eğer hedef_x ve rx birbirine çok yakınsa (fark < 2m) araç zaten ulaştım sanır .
        fark_x = abs(hedef_x - rx)
        fark_z = abs(hedef_z - rz)
        print(f"  Hedef farkı → dX={fark_x:.2f}m  dZ={fark_z:.2f}m")
        if fark_x < 3.0 and fark_z < 3.0:
            print("UYARI: Hedef çok yakın! REF_LAT/REF_LNG'yi kontrol et.")

    # 3b) KOMUT (PAUSE / RUN / CLEAR)
    command = server.poll_command(counter)
    if command == 'PAUSE':
        nav_state = 'PAUSED'
        driver.setCruisingSpeed(0.0)
        driver.setSteeringAngle(0.0)
        driver.setBrakeIntensity(0.5)
        current_speed = 0.0
        steer_pid.reset()
        speed_pid.reset()
        print("ARAÇ DURAKLATILDI")
    elif command == 'RUN' and nav_state == 'PAUSED':
        nav_state = 'NORMAL'
        driver.setBrakeIntensity(0.0)
        steer_pid.reset()
        speed_pid.reset()
        print("ARAÇ DEVAM EDİYOR")
    elif command == 'CLEAR':
        hedef_x = hedef_z = hedef_lat = hedef_lng = None
        hedefe_ulasti    = False
        nav_state        = 'NORMAL'
        clear_step_count = 0
        driver.setCruisingSpeed(0.0)
        driver.setSteeringAngle(0.0)
        driver.setBrakeIntensity(0.0)
        current_speed = 0.0
        steer_pid.reset()
        speed_pid.reset()
        print("ROTA TEMİZLENDİ")

    # 4) TELEMETRİ
    dist_to_wp = haversine_m(robot_lat, robot_lng, hedef_lat, hedef_lng) \
                 if hedef_lat else 0.0
    server.send_telemetry(counter, robot_lat, robot_lng, heading,
                          current_speed, dist_to_wp, nav_state, obstacles)

    # 5) HEDEF YOK / DURAKLATILDI
    if hedef_x is None or hedefe_ulasti or nav_state == 'PAUSED':
        driver.setCruisingSpeed(0.0)
        driver.setSteeringAngle(0.0)
        driver.setBrakeIntensity(0.0)
        current_speed = 0.0
        continue

    # 6) NAVİGASYON
    angle_err, dist_m, arrived = compute_nav_errors(rx, rz, heading, hedef_x, hedef_z)
    desired_spd   = target_speed(dist_m, angle_err)
    gps_avoid_dir = avoid_direction(rx, rz, heading, hedef_x, hedef_z)

    # 7) LOKAL PLANLAMA
    decision = local_planner.decide(ds_dist, obstacles, gps_avoid_dir)

    # TEŞHİS ÇIKTISI
    if DEBUG_EVERY > 0 and debug_counter >= DEBUG_EVERY:
        debug_counter = 0
        blk_c = obstacles.get('center_blocked', obstacles.get('center', 0) > 30)
        print(
            f"[D] mod={nav_state:<8} dist={dist_m:6.1f}m "
            f"angle={angle_err:+.2f}r  spd_istek={desired_spd:5.1f}km/h "
            f"ds={ds_dist:5.2f}m  karar={decision['mode']:<10} "
            f"arrived={arrived}  cam_merkez_engel={blk_c}"
        )

        if desired_spd < 0.5:
            print(f"SORUN: desired_spd={desired_spd:.2f}  dist={dist_m:.1f}  angle={angle_err:.2f}")
            print(f"  dist küçükse REF koordinatı yanlış, angle büyükse yön sorunu")
        if decision['mode'] == 'EMERGENCY':
            print(f"SORUN: ACİL FREN aktif — ds0={ds_dist:.2f}m  (planner eşiği: 0.8m)")
            print(f"  ds0 lookup table'ı olmayan sensörde ham 0 geliyor olabilir")

    # 8) DURUM MAKİNESİ

    # ACİL FREN
    if decision['mode'] == 'EMERGENCY':
        if ds_dist < 0.5:
            nav_state = 'EMERGENCY'
            driver.setBrakeIntensity(1.0)
            driver.setCruisingSpeed(0.0)
            driver.setSteeringAngle(0.0)
            steer_pid.reset()
            speed_pid.reset()
            current_speed = 0.0
            print(decision['reason'])
        else:
            decision['mode'] = 'NORMAL'

    # GERİ GİT
    if decision['mode'] == 'REVERSE':
        nav_state = 'AVOIDING'
        driver.setBrakeIntensity(0.0)
        driver.setCruisingSpeed(decision['speed'])
        driver.setSteeringAngle(decision['steer'])
        current_speed = abs(decision['speed'])

    # ENGEL KAÇINMA
    elif decision['mode'] == 'AVOID':
        if nav_state == 'NORMAL':
            stored_avoid_dir = gps_avoid_dir
            clear_step_count = 0
            steer_pid.reset()
            speed_pid.reset()
            print(f"ENGEL!  ds={ds_dist:.2f}m | {decision['reason']}")
        nav_state = 'AVOIDING'
        driver.setBrakeIntensity(0.0)
        driver.setCruisingSpeed(decision['speed'])
        driver.setSteeringAngle(decision['steer'])
        current_speed = decision['speed']

    # NORMAL / PID
    elif decision['mode'] in ('NORMAL', 'EMERGENCY'):

        # Kaçınmadan çıkış
        if nav_state == 'AVOIDING':
            clear_step_count = (clear_step_count + 1) if ds_dist >= DS_CLEAR_DIST else 0
            if clear_step_count >= CLEAR_STEPS_NEEDED and \
               facing_target(rx, rz, heading, hedef_x, hedef_z):
                nav_state = 'NORMAL'
                clear_step_count = 0
                steer_pid.reset()
                speed_pid.reset()
                print("Engel geçildi → Normal mod")
            else:
                driver.setBrakeIntensity(0.0)
                driver.setCruisingSpeed(AVOID_SPEED_KMH)
                driver.setSteeringAngle(stored_avoid_dir * AVOID_STEER)
                current_speed = AVOID_SPEED_KMH
                continue

        # Waypoint'e ulaşıldı
        if arrived:
            print(f"Waypoint!  GPS: {robot_lat:.6f}, {robot_lng:.6f}")
            driver.setBrakeIntensity(0.5)
            driver.setCruisingSpeed(0.0)
            driver.setSteeringAngle(0.0)
            steer_pid.reset()
            speed_pid.reset()
            current_speed = 0.0
            server.notify_waypoint_reached(hedef_lat, hedef_lng)
            hedef_x = hedef_z = hedef_lat = hedef_lng = None
            hedefe_ulasti = True
            continue

        # PID — direksiyon
        steer_out = steer_pid.compute(angle_err, dt)

        # PID — hız
        speed_delta = speed_pid.compute(desired_spd - current_speed, dt)
        new_speed   = clamp(current_speed + speed_delta, 0.0, MAX_SPEED_KMH)

        nav_state     = 'NORMAL'
        current_speed = new_speed

        driver.setBrakeIntensity(0.0)
        driver.setSteeringAngle(steer_out)
        driver.setCruisingSpeed(new_speed)

vision.close()