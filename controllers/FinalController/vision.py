import cv2
import numpy as np

# ROI AYARLARI 
# Araç tam önündeki 1-2 m engel kamerada AŞAĞI görünür.
ROI_Y_START = 0.40
ROI_Y_END   = 0.95

# CANNY AYARLARI 
CANNY_LOW       = 30    # eskiden 50
CANNY_HIGH      = 100   # eskiden 150
CANNY_THRESHOLD = 30    # bu piksel sayısının üstü = engel (eskiden 50)

# RENK MASKESI 
# (HSV_min, HSV_max)
COLOR_RANGES = [
    ((5,  100, 80),  (25, 255, 255)),   # Turuncu
    ((0,  120, 80),  (10, 255, 255)),   # Kırmızı alt
    ((160,120, 80),  (180,255, 255)),   # Kırmızı üst
    ((25,  80, 80),  (40, 255, 255)),   # Sarı
    ((100, 80, 80),  (130,255, 255)),   # Mavi
]
COLOR_PIXEL_THRESHOLD = 200

# KONTUR AYARLARI
CONTOUR_MIN_AREA = 400 

# GÖRÜNTÜ
DISPLAY_SCALE = 2
SHOW_CAMERA   = True

OBSTACLE_THRESHOLD = CANNY_THRESHOLD

class VisionSystem:
    def analyze(self, camera):
        try:
            raw = camera.getImage()
            if not raw:
                return self._empty()

            img   = np.frombuffer(raw, np.uint8).reshape(
                        (camera.getHeight(), camera.getWidth(), 4))[:, :, :3]
            h, w  = img.shape[:2]
            y1    = int(h * ROI_Y_START)
            y2    = int(h * ROI_Y_END)
            third = w // 3
            roi   = img[y1:y2, :]

            # Canny
            gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), CANNY_LOW, CANNY_HIGH)
            canny = {
                'left':   int(np.count_nonzero(edges[:, 0:third])),
                'center': int(np.count_nonzero(edges[:, third:2*third])),
                'right':  int(np.count_nonzero(edges[:, 2*third:]))
            }

            # Renk
            hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = np.zeros(roi.shape[:2], dtype=np.uint8)
            for lo, hi in COLOR_RANGES:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lo), np.array(hi)))
            k    = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            color = {
                'left':   int(np.count_nonzero(mask[:, 0:third])),
                'center': int(np.count_nonzero(mask[:, third:2*third])),
                'right':  int(np.count_nonzero(mask[:, 2*third:]))
            }

            # Kontur alanı
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cf = {'left': False, 'center': False, 'right': False}
            for cnt in cnts:
                if cv2.contourArea(cnt) < CONTOUR_MIN_AREA:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                if   cx < third:     cf['left']   = True
                elif cx < 2*third:   cf['center'] = True
                else:                cf['right']  = True

            # Karar
            def blocked(z):
                return (canny[z] > CANNY_THRESHOLD or
                        color[z] > COLOR_PIXEL_THRESHOLD or
                        cf[z])

            obstacles = {
                'left':            canny['left'],
                'center':          canny['center'],
                'right':           canny['right'],
                'left_blocked':    blocked('left'),
                'center_blocked':  blocked('center'),
                'right_blocked':   blocked('right'),
                'color_left':      color['left'],
                'color_center':    color['center'],
                'color_right':     color['right'],
            }

            dbg = self._debug(img, edges, mask, obstacles, y1, y2, third, w, cnts) \
                  if SHOW_CAMERA else None
            return obstacles, dbg

        except Exception as e:
            print(f"[VisionSystem] Hata: {e}")
            return self._empty()

    def show(self, debug_img, nav_state, ds_dist, speed):
        if debug_img is None or not SHOW_CAMERA:
            return
        cv2.putText(debug_img,
            f"MOD:{nav_state}  ds:{ds_dist:.2f}m  {speed:.1f}km/h",
            (8, debug_img.shape[0] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
        cv2.imshow("Otonom Arac - Goruntu", debug_img)
        cv2.waitKey(1)

    def close(self):
        if SHOW_CAMERA:
            cv2.destroyAllWindows()

    # Yardımcılar
    def _empty(self):
        return {
            'left': 0, 'center': 0, 'right': 0,
            'left_blocked': False, 'center_blocked': False, 'right_blocked': False,
            'color_left': 0, 'color_center': 0, 'color_right': 0
        }, None

    def _debug(self, orig, edges, mask, obs, y1, y2, third, w, cnts):
        rh = y2 - y1

        # Sol panel: Canny  ,  Sağ panel: Renk maskesi
        left_panel  = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        right_panel = np.zeros((rh, w, 3), dtype=np.uint8)
        right_panel[:, :, 1] = mask   # yeşil kanalda maskeli bölge
        cv2.drawContours(right_panel, cnts, -1, (0, 200, 255), 1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        zones = [
            ('L', 0,       'left'),
            ('C', third,   'center'),
            ('R', 2*third, 'right'),
        ]
        for name, x0, key in zones:
            x1b  = x0 + third - 1
            blk  = obs[f'{key}_blocked']
            bcol = (0, 40, 200) if blk else (40, 100, 40)

            # Bölge çerçeveleri
            cv2.rectangle(left_panel,  (x0, 0), (x1b, rh-1), bcol, 2)
            cv2.rectangle(right_panel, (x0, 0), (x1b, rh-1), bcol, 2)
            cv2.line(left_panel,  (x0+third//2, 0), (x0+third//2, rh), (60,60,60), 1)

            # Canny sayısı
            cv2.putText(left_panel,
                f"{name}:{obs[key]}",
                (x0+4, 20), font, 0.5,
                (0,50,255) if blk else (180,255,180), 1)
            # Renk sayısı
            cv2.putText(right_panel,
                f"{name}:{obs[f'color_{key}']}",
                (x0+4, 20), font, 0.5,
                (0,200,255) if blk else (160,255,160), 1)
            # ENGELLİ etiketi
            if blk:
                cv2.putText(left_panel,  "ENGEL", (x0+4, rh-6), font, 0.45, (0,50,255), 1)
                cv2.putText(right_panel, "ENGEL", (x0+4, rh-6), font, 0.45, (0,200,255),1)

        # ROI kutusunu orijinal görüntüde göster
        roi_preview = orig.copy()
        cv2.rectangle(roi_preview, (0, y1), (w-1, y2-1), (0, 220, 0), 2)
        roi_small   = cv2.resize(roi_preview, (w * 2, max(rh // 2, 20)))

        double_panel = np.hstack([left_panel, right_panel])
        full         = np.vstack([roi_small, double_panel])

        return cv2.resize(full,
            (full.shape[1] * DISPLAY_SCALE // 2,
             full.shape[0] * DISPLAY_SCALE // 2),
            interpolation=cv2.INTER_LINEAR)