from vehicle import Driver
import requests
import math
import threading

# ─── AYARLAR ──────────────────────────────────────────────────────────────────
SERVER_URL = "http://192.168.1.142:5000"

REF_LAT = 38.71402447794388
REF_LNG = 35.565990955616584

MAX_SPEED_KMH    = 30.0   # km/h — normal seyir hızı
AVOID_SPEED_KMH  = 10.0   # km/h — engel kaçınma hızı
MAX_STEER_ANGLE  = 0.5    # radyan
ARRIVE_THRESHOLD = 2.0    # metre

K_STEER = 1.2
K_SPEED = 0.8

TARGET_POLL_EVERY    = 50
TELEMETRY_SEND_EVERY = 30

# ─── ENGEL ALGILAMA AYARLARI ──────────────────────────────────────────────────
OBSTACLE_DETECT_DIST = 3.0   # metre — bu mesafede engel "var" sayılır
OBSTACLE_CLEAR_DIST  = 4.5   # metre — bu mesafenin üstünde engel "yok" sayılır
                              # (histerezis: titreme önlemek için detect'ten büyük)

# Kaçınma direksiyon açısı — engel sağda/solda tam dönüş
AVOID_STEER = 0.45            # radyan (~26°)

# Engel geçildikten sonra ne kadar "serbest" adım sayılırsa rota moduna dön
CLEAR_STEPS_NEEDED = 40


# ─── KOORDİNAT DÖNÜŞÜM ───────────────────────────────────────────────────────

def latlon_to_meters(target_lat, target_lng):
    lat_diff = target_lat - REF_LAT
    lng_diff = target_lng - REF_LNG
    meter_x  = lat_diff * 111111
    meter_z  = lng_diff * 111111 * math.cos(math.radians(REF_LAT))
    return meter_x, meter_z


def meters_to_latlon(meter_x, meter_z):
    lat = REF_LAT + meter_x / 111111
    lng = REF_LNG + meter_z / (111111 * math.cos(math.radians(REF_LAT)))
    return lat, lng


def get_heading_from_compass(compass_values):
    return math.atan2(compass_values[0], compass_values[2])


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


def haversine_m(lat1, lng1, lat2, lng2):
    R    = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lng2 - lng1)
    a    = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ─── NAVİGASYON (normal mod) ──────────────────────────────────────────────────

def navigate_to(robot_x, robot_z, heading, target_x, target_z):
    """Hedefe P-kontrol ile git. Döndürür: (steer, speed_kmh, ulaşıldı_mı)"""
    dx       = target_x - robot_x
    dz       = target_z - robot_z
    distance = math.sqrt(dx**2 + dz**2)

    if distance < ARRIVE_THRESHOLD:
        return 0.0, 0.0, True

    target_angle = math.atan2(dz, dx)
    angle_diff   = target_angle - heading
    while angle_diff >  math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    steer       = clamp(K_STEER * angle_diff, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)
    turn_factor = 1.0 - (abs(steer) / MAX_STEER_ANGLE) * 0.6
    dist_factor = clamp(K_SPEED * distance / 10.0, 0.2, 1.0)
    speed       = MAX_SPEED_KMH * turn_factor * dist_factor

    return steer, speed, False


# ─── ENGEL KAÇINMA (Bug algoritması) ─────────────────────────────────────────
#
# Mantık:
#   NORMAL mod  → navigate_to() ile hedefe git
#   AVOIDING mod → engel algılandı:
#       1. Hedef hangi yönde? → hedefe göre sol/sağ karar ver
#       2. O yönde sabit direksiyon açısıyla ilerle (engeli dolaş)
#       3. Önü açık VE hedefe doğru ilerliyorsa → NORMAL'e dön
#
# Tek sensör (ds0 = ön merkez) ile çalışır.
# Birden fazla sensörün olduğu durumda genişletilebilir.

def decide_avoid_direction(robot_x, robot_z, heading, target_x, target_z):
    """
    Engeli soldan mı sağdan mı dolaşalım?
    Hedefe olan açıya göre karar verilir:
      - Hedef solda ise sağdan dolaş (sol boşluğu hedefe sakla)
      - Hedef sağda ise soldan dolaş
    Döndürür: +1 (sola dön) veya -1 (sağa dön)
    """
    dx           = target_x - robot_x
    dz           = target_z - robot_z
    target_angle = math.atan2(dz, dx)
    angle_diff   = target_angle - heading
    while angle_diff >  math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi

    # angle_diff > 0 → hedef solda → sağdan dolaş (-1)
    # angle_diff < 0 → hedef sağda → soldan dolaş (+1)
    return -1 if angle_diff > 0 else 1


def is_heading_toward_target(robot_x, robot_z, heading, target_x, target_z,
                              tolerance_rad=0.6):
    """
    Araç başlığı hedefe yeterince yakın mı bakıyor?
    Engel bittikten sonra rota moduna dönüş kararı için kullanılır.
    """
    dx           = target_x - robot_x
    dz           = target_z - robot_z
    target_angle = math.atan2(dz, dx)
    angle_diff   = target_angle - heading
    while angle_diff >  math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi
    return abs(angle_diff) < tolerance_rad


# ─── ARKA PLAN HTTP ───────────────────────────────────────────────────────────

def post_async(endpoint, payload):
    def _send():
        try:
            requests.post(SERVER_URL + endpoint, json=payload, timeout=0.3)
        except Exception:
            pass
    threading.Thread(target=_send, daemon=True).start()


# ─── ROBOT BAŞLATMA ───────────────────────────────────────────────────────────

driver   = Driver()
timestep = int(driver.getBasicTimeStep())

print("=== Cihaz Listesi ===")
for i in range(driver.getNumberOfDevices()):
    print(f"  {i}: {driver.getDeviceByIndex(i).getName()}")
print("=====================")

# GPS
gps = driver.getDevice("gps")
if gps:
    gps.enable(timestep)
    print("✓ GPS aktif")
else:
    print("✗ HATA: 'gps' bulunamadı!")

# Compass
compass = driver.getDevice("compass")
if compass:
    compass.enable(timestep)
    print("✓ Compass aktif")
else:
    print("✗ HATA: 'compass' bulunamadı!")

# Distance Sensor — ds0
ds = driver.getDevice("ds0")
if ds:
    ds.enable(timestep)
    print("✓ DistanceSensor (ds0) aktif")
else:
    print("✗ HATA: 'ds0' bulunamadı!")

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)

# ─── DURUM DEĞİŞKENLERİ ───────────────────────────────────────────────────────
hedef_x        = None
hedef_z        = None
hedef_lat      = None
hedef_lng      = None
hedefe_ulasti  = False
current_speed  = 0.0
counter        = 0

# Engel kaçınma durumu
# 'NORMAL'   → hedefe git
# 'AVOIDING' → engeli dolaş
avoid_state      = 'NORMAL'
avoid_direction  = 1        # +1 sol, -1 sağ
clear_step_count = 0        # engel bittikten kaç adım geçti

print(f"\nAraç hazır. Sunucu: {SERVER_URL}\n")

# ─── ANA DÖNGÜ ────────────────────────────────────────────────────────────────

while driver.step() != -1:
    counter += 1

    # ── 1) SENSÖR OKUMA ───────────────────────────────────────────────────────
    if gps:
        gps_vals = gps.getValues()
        robot_x  = gps_vals[0]
        robot_z  = gps_vals[2]
    else:
        robot_x, robot_z = 0.0, 0.0

    heading = get_heading_from_compass(compass.getValues()) if compass else 0.0

    # DistanceSensor değeri: küçük değer = yakın engel
    # Webots DistanceSensor genellikle 0‒1000 arası ham değer döner.
    # lookup table yoksa değeri doğrudan metre olarak yorumlamak için
    # sensörün maxRange ayarına göre normalize edilir.
    # Senin sensörün ham metre dönüyorsa ds_dist = ds.getValue() yeterli.
    # Ham 0-1000 dönüyorsa: ds_dist = ds.getValue() / 1000 * MAX_RANGE
    # Aşağıda direkt metre varsayıldı — gerekirse ayarla.
    if ds:
        ds_dist = ds.getValue()   # metre cinsinden mesafe varsayımı
    else:
        ds_dist = 999.0           # sensör yoksa engel yok say

    robot_lat, robot_lng = meters_to_latlon(robot_x, robot_z)

    # ── 2) SUNUCUDAN HEDEF SORGULA ────────────────────────────────────────────
    if counter % TARGET_POLL_EVERY == 0:
        try:
            r    = requests.get(SERVER_URL + "/get_target", timeout=0.1)
            data = r.json()
            if data.get("lat") is not None and data.get("updated") == True:
                hedef_lat, hedef_lng = data["lat"], data["lng"]
                hedef_x, hedef_z     = latlon_to_meters(hedef_lat, hedef_lng)
                hedefe_ulasti        = False
                avoid_state          = 'NORMAL'   # yeni hedef → kaçınmayı sıfırla
                clear_step_count     = 0
                print(f"▶ Yeni hedef  GPS:    {hedef_lat:.6f}, {hedef_lng:.6f}")
                print(f"              Webots: X={hedef_x:.2f}m  Z={hedef_z:.2f}m")
        except Exception:
            pass

    # ── 3) TELEMETRİ ──────────────────────────────────────────────────────────
    if counter % TELEMETRY_SEND_EVERY == 0:
        dist = haversine_m(robot_lat, robot_lng, hedef_lat, hedef_lng) \
               if hedef_lat is not None else 0.0
        post_async("/vehicle_update", {
            "lat":               robot_lat,
            "lng":               robot_lng,
            "heading":           heading,
            "speed":             current_speed,
            "dist_to_target":    dist,
            "waypoints_remaining": 0,
            "obstacle_mode":     avoid_state   # haritada göstermek için bonus
        })

    # ── 4) HEDEF YOK → bekle ──────────────────────────────────────────────────
    if hedef_x is None or hedefe_ulasti:
        driver.setCruisingSpeed(0.0)
        driver.setSteeringAngle(0.0)
        current_speed = 0.0
        continue

    # ── 5) ENGEL KAÇINMA DURUM MAKİNESİ ──────────────────────────────────────

    if avoid_state == 'NORMAL':
        # ── Normal mod: hedefe git ─────────────────────────────────────────
        steer, speed_kmh, hedefe_ulasti = navigate_to(
            robot_x, robot_z, heading, hedef_x, hedef_z
        )

        if ds_dist < OBSTACLE_DETECT_DIST:
            # ENGEL ALGILANDI → kaçınma moduna geç
            avoid_direction  = decide_avoid_direction(
                robot_x, robot_z, heading, hedef_x, hedef_z
            )
            avoid_state      = 'AVOIDING'
            clear_step_count = 0
            side             = "SOLA" if avoid_direction > 0 else "SAĞA"
            print(f"⚠ ENGEL! Mesafe: {ds_dist:.2f}m → {side} kaçınma başladı")

            driver.setSteeringAngle(avoid_direction * AVOID_STEER)
            driver.setCruisingSpeed(AVOID_SPEED_KMH)
            current_speed = AVOID_SPEED_KMH

        else:
            # Normal seyir
            driver.setSteeringAngle(steer)
            driver.setCruisingSpeed(speed_kmh)
            current_speed = speed_kmh

            if hedefe_ulasti:
                print(f"✓ Waypoint'e ulaşıldı! GPS: {robot_lat:.6f}, {robot_lng:.6f}")
                driver.setCruisingSpeed(0.0)
                driver.setSteeringAngle(0.0)
                current_speed = 0.0
                post_async("/waypoint_reached",
                           {"waypoint": {"lat": hedef_lat, "lng": hedef_lng}})
                hedef_x = hedef_z = hedef_lat = hedef_lng = None

    elif avoid_state == 'AVOIDING':
        # ── Kaçınma modu: engeli dolaş ────────────────────────────────────

        if ds_dist >= OBSTACLE_CLEAR_DIST:
            # Önü açık — sayaç artır
            clear_step_count += 1
        else:
            # Hâlâ engel var — sayacı sıfırla, dönmeye devam
            clear_step_count = 0

        if clear_step_count >= CLEAR_STEPS_NEEDED:
            # Yeterince açık alan geçti VE hedefe bakıyorsa normal moda dön
            if hedef_x is not None and is_heading_toward_target(
                    robot_x, robot_z, heading, hedef_x, hedef_z):
                avoid_state      = 'NORMAL'
                clear_step_count = 0
                print("✓ Engel geçildi → Normal navigasyon moduna dönüldü")
            else:
                # Henüz hedefe bakmıyor — biraz daha dön
                clear_step_count = max(0, clear_step_count - 5)

        # Engel hâlâ çok yakınsa frene bas (tamamen durma)
        if ds_dist < OBSTACLE_DETECT_DIST * 0.6:
            # Çok yakın — dur ve sert dön
            driver.setCruisingSpeed(AVOID_SPEED_KMH * 0.5)
            driver.setSteeringAngle(avoid_direction * MAX_STEER_ANGLE)
        else:
            # Orta mesafe — yavaş ilerle ve dön
            driver.setCruisingSpeed(AVOID_SPEED_KMH)
            driver.setSteeringAngle(avoid_direction * AVOID_STEER)

        current_speed = AVOID_SPEED_KMH