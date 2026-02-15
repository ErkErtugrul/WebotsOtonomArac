import math

# REFERANS NOKTA
# Webots (0, 0, 0) 
REF_LAT = 0
REF_LNG = 0

# ARAÇ SABİTLERİ 
MAX_SPEED_KMH = 60.0   # km/h — düz yolda maksimum
MAX_STEER_ANGLE = 0.5    # radyan
ARRIVE_THRESHOLD = 2.0    # metre — waypoint ulaşıldı eşiği

#  PID KATSAYILARI 
STEER_KP = 1.0
STEER_KI = 0.02
STEER_KD = 0.15
STEER_I_MAX = 0.3    # radyan

# Hız: hata = (hedef_hız - mevcut_hız) km/h → çıkış = hız deltası (km/h)
SPEED_KP = 1.2   
SPEED_KI = 0.05   
SPEED_KD = 0.10   
SPEED_I_MAX = 20.0   

class PIDController:
    def __init__(self, kp, ki, kd, out_min, out_max, i_max=None):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.out_min = out_min
        self.out_max = out_max
        self.i_max   = i_max if i_max is not None else abs(out_max)

        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_run  = True

    def compute(self, error, dt):
        """
        error : ölçülen hata  (hedef − mevcut)
        dt    : adım süresi saniye cinsinden  (timestep / 1000)
        """
        if dt <= 0:
            dt = 1e-4

        if self._first_run:
            self._prev_error = error
            self._first_run  = False

        # P
        p_term = self.kp * error

        # I  (anti-windup)
        self._integral += error * dt
        self._integral  = max(-self.i_max, min(self.i_max, self._integral))
        i_term = self.ki * self._integral

        # D  (sonlu fark)
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        return max(self.out_min, min(self.out_max, p_term + i_term + d_term))

    def reset(self):
        """Yeni waypoint veya mod değişiminde çağır."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_run  = True


# KOORDİNAT DÖNÜŞÜM
def latlon_to_meters(lat, lng):
    """GPS (derece) → Webots dünya koordinatı (metre)."""
    return (
        (lat - REF_LAT) * 111111,
        (lng - REF_LNG) * 111111 * math.cos(math.radians(REF_LAT))
    )


def meters_to_latlon(x, z):
    """Webots dünya koordinatı (metre) → GPS (derece)."""
    return (
        REF_LAT + x / 111111,
        REF_LNG + z / (111111 * math.cos(math.radians(REF_LAT)))
    )


def haversine_m(lat1, lng1, lat2, lng2):
    """İki GPS noktası arası gerçek mesafe (metre)."""
    R  = 6371000
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lng2 - lng1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# KONUM YARDIMCILARI
def get_heading(compass_vals):
    # Compass sensörü değerinden araç başlık açısı (radyan)
    return math.atan2(compass_vals[0], compass_vals[2])


def normalize_angle(a):
    """Açıyı −π ile +π arasına uyarla."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def compute_nav_errors(rx, rz, heading, tx, tz):
    """
    Mevcut konum + başlık → hedef noktasına göre hata hesaplar.

    Döndürür:
        angle_diff  : direksiyon hatası (radyan)  — PID girişi
        distance    : hedefe kalan mesafe (metre)
        arrived     : bool — ARRIVE_THRESHOLD altına düştü mü
    """
    dx, dz = tx - rx, tz - rz
    dist   = math.sqrt(dx**2 + dz**2)

    if dist < ARRIVE_THRESHOLD:
        return 0.0, dist, True

    angle_diff = normalize_angle(math.atan2(dz, dx) - heading)
    return angle_diff, dist, False


def target_speed(dist, angle_diff):
    """
    Anlık duruma göre hedef hız (km/h) belirler.

    dist_factor : 5m'den uzakta TAM HIZ, 5m altında kademeli yavaşlama
    turn_factor : sadece keskin dönüşlerde (>45°) hız kısılır
    """
    # 5m altında yavaşla (eskiden 15m idi — çok erkendi)
    dist_factor = clamp(dist / 5.0, 0.25, 1.0)

    # 45° (0.78 rad) altında hıza dokunma
    angle_abs = abs(angle_diff)
    if angle_abs < 0.78:
        turn_factor = 1.0
    else:
        turn_factor = 1.0 - ((angle_abs - 0.78) / (math.pi - 0.78)) * 0.6
        turn_factor = max(turn_factor, 0.4)

    return MAX_SPEED_KMH * dist_factor * turn_factor


def avoid_direction(rx, rz, heading, tx, tz):
    """
    
    Engeli hangi yönden dolaşmalı?
    
    """
    diff = normalize_angle(math.atan2(tz - rz, tx - rx) - heading)
    return -1 if diff > 0 else 1


def facing_target(rx, rz, heading, tx, tz, tol=0.55):
    """Araç hedefe bakıyor mu? (radyan)
    """
    return abs(normalize_angle(math.atan2(tz - rz, tx - rx) - heading)) < tol