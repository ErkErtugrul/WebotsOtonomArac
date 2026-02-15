from vision import OBSTACLE_THRESHOLD as CAM_THRESHOLD

DS_EMERGENCY_DIST  = 0.8
DS_AVOID_DIST      = 2.5
DS_CLEAR_DIST      = 4.0
AVOID_SPEED_KMH    = 10.0
AVOID_STEER        = 0.45
CLEAR_STEPS_NEEDED = 45

class LocalPlanner:
    """
    Karar önceliği:
      1. ds < DS_EMERGENCY_DIST          → ACİL FREN
      2. ds < DS_AVOID_DIST              → KAÇINMA
            a. center_blocked=False      → Yavaşla, yön GPS'e
            b. left_blocked=False        → Sola dön
            c. right_blocked=False       → Sağa dön
            d. GPS yönü                  → GPS yönünde dön
            e. Hiçbiri                   → Geri git
      3. ds >= DS_AVOID_DIST             → NORMAL
    """

    def decide(self, ds_dist, obstacles, gps_avoid_dir):
        use_blocked = 'center_blocked' in obstacles
        if use_blocked:
            left_ok   = not obstacles['left_blocked']
            center_ok = not obstacles['center_blocked']
            right_ok  = not obstacles['right_blocked']
        else:
            left_ok   = obstacles.get('left',   0) < CAM_THRESHOLD
            center_ok = obstacles.get('center', 0) < CAM_THRESHOLD
            right_ok  = obstacles.get('right',  0) < CAM_THRESHOLD

        # ACİL FREN
        if ds_dist < DS_EMERGENCY_DIST:
            return {'mode': 'EMERGENCY', 'steer': 0.0, 'speed': 0.0,
                    'brake': 1.0, 'reason': f'ACİL FREN!  ds={ds_dist:.2f}m'}

        # KAÇINMA MODU
        if ds_dist < DS_AVOID_DIST:
            if center_ok:
                return {'mode': 'AVOID', 'steer': 0.0,
                        'speed': AVOID_SPEED_KMH, 'brake': 0.0,
                        'reason': 'Yavaşlıyor — merkez temiz'}
            if left_ok:
                return {'mode': 'AVOID', 'steer': -AVOID_STEER,
                        'speed': AVOID_SPEED_KMH, 'brake': 0.0,
                        'reason': 'Sola kaçıyor (kamera)'}
            if right_ok:
                return {'mode': 'AVOID', 'steer': AVOID_STEER,
                        'speed': AVOID_SPEED_KMH, 'brake': 0.0,
                        'reason': 'Sağa kaçıyor (kamera)'}
            if gps_avoid_dir != 0:
                return {'mode': 'AVOID',
                        'steer': gps_avoid_dir * AVOID_STEER,
                        'speed': AVOID_SPEED_KMH, 'brake': 0.0,
                        'reason': f'GPS yönünde ({gps_avoid_dir:+d})'}
            return {'mode': 'REVERSE', 'steer': 0.3, 'speed': -8.0,
                    'brake': 0.0, 'reason': 'Geri gidiyor'}

        # NORMAL
        return {'mode': 'NORMAL', 'steer': None, 'speed': None,
                'brake': 0.0, 'reason': 'Normal seyir'}