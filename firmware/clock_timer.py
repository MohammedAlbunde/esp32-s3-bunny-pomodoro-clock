# Digital Clock + Timer (MicroPython) - Waveshare ESP32-S3-Touch-LCD-1.28
#
# CLOCK screen : HH:MM (big), seconds, date. Time comes from WiFi (NTP).
# TIMER screen : countdown timer (1-99 minutes).
#
# TOUCH GUIDE
#   Clock screen : tap middle -> timer screen
#                  tap left / right -> other pages (weather, Baghdad time)
#   Timer (set)  : tap left side = -1 min, right side = +1 min (hold = repeat)
#                  tap the middle = START, hold the middle = back to clock
#   Timer (run)  : tap the middle = pause / resume
#                  tap left or right side = go to the clock (timer keeps running)
#                  hold = stop the timer
#   Time is up   : screen flashes, tap = close, hold = clock
#   Clock screen : HOLD = settings (style, theme, 12/24h, brightness, night dim)
#   Settings     : tap middle = change, tap left/right = previous/next item, hold = done
#   BOOT button  : works like tapping the middle
#   Weather / Baghdad page : tap left / right = next page, tap middle or hold = clock
#   Shake the board : wakes the screen and closes the timer alarm
#   (AUTO ROTATE in the settings turns the screen upside down when the board is)
#   Clock: tap LEFT = ALARM page.  Pages to the right: weather, Baghdad, BATTERY, ASK AI.
#   ALARM page : sides = change, middle = next field (ON/OFF, hour, minutes), hold = save
#   Alarm rings: tap = stop, shake = snooze 5 min.  (optional buzzer: set ALARM_PIN)
#   FLIP TIMER (settings): set minutes on the timer page, lay the board face-down to start,
#     lift it to pause, face-down again to continue. Screen is off while face-down.
#   ASK AI page : open http://<clock ip>/update on a PC/phone to upload a new clock_timer.py
#
# WIFI SETUP (automatic)
#   If the saved WiFi cannot be reached, the clock shows the nearby networks and
#   opens a setup WiFi (Clock-Setup). Join it from your phone, open the address
#   shown on the screen, pick your network and type its password. Saved in /wifi.json.
#   Hold the screen to cancel.

import time
import gc
import framebuf
from array import array
from machine import Pin, SPI, I2C, PWM
from math import sin, cos, radians
import struct

# =====================================================================
#  SETTINGS - edit these
# =====================================================================
WIFI_SSID = "YOUR_WIFI_NAME"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"   # (optional) /wifi.json from the setup page wins
SETUP_AP_NAME = "Clock-Setup"   # name of the temporary setup WiFi
SETUP_TIMEOUT_MIN = 10          # give up the setup screen after N minutes
FAILS_BEFORE_SETUP = 3          # failed re-syncs in a row before the setup screen opens

TZ_STD_OFFSET = -5        # Montreal = UTC-5 (winter). Summer time is added automatically.
USE_DST = True            # North America daylight saving (2nd Sun Mar -> 1st Sun Nov)
MANUAL_OFFSET_HOURS = 0   # only used when WiFi/NTP is not working
RESYNC_HOURS = 6          # re-sync the time every N hours
USE_24H = True            # default format (can be changed in the settings screen)
NIGHT_START = 22          # night dim starts at this hour (local time)
NIGHT_END = 7             # ... and ends at this hour
NIGHT_PCT = 8             # brightness (%) at night when NIGHT DIM is ON
WAKE_MS = 10000           # touching the screen at night lights it for this long
DEFAULT_TIMER_MIN = 5     # starting timer length (minutes)
WEATHER_LAT = 45.50       # weather location (Montreal)
WEATHER_LON = -73.57
WEATHER_CITY = "MONTREAL"
WEATHER_MIN = 30          # refresh the weather every N minutes
WORLD_CITY = "BAGHDAD"    # second clock
WORLD_OFFSET = 3          # Baghdad = UTC+3 all year (no daylight saving)
IMU_UP_SIGN = 1           # set -1 if AUTO ROTATE turns the screen the wrong way
SHAKE_G = 2.2             # how hard a shake must be
MOTION_G = 0.15           # how much movement wakes a dimmed screen

SWAP_RB = True            # red/blue fix
MIRROR_X = True           # left/right mirror fix (display)
FLIP_Y = False            # top/bottom flip (display)
TOUCH_FLIP_X = False      # set True if "-" and "+" sides are swapped when you tap

# =====================================================================
#  pins (Waveshare ESP32-S3-Touch-LCD-1.28)
# =====================================================================
LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, LCD_RST, LCD_BL = 8, 9, 10, 11, 14, 2
TP_SDA, TP_SCL, TP_RST = 6, 7, 13
TOUCH_ADDR = 0x15
BUTTON_PIN = 0

MADCTL = (0x00 if SWAP_RB else 0x08) | (0x40 if MIRROR_X else 0) | (0x80 if FLIP_Y else 0)

# =====================================================================
#  GC9A01 display driver
# =====================================================================
_INIT = (
    (0xEF, b"", 0), (0xEB, b"\x14", 0), (0xFE, b"", 0), (0xEF, b"", 0),
    (0xEB, b"\x14", 0), (0x84, b"\x40", 0), (0x85, b"\xFF", 0),
    (0x86, b"\xFF", 0), (0x87, b"\xFF", 0), (0x88, b"\x0A", 0),
    (0x89, b"\x21", 0), (0x8A, b"\x00", 0), (0x8B, b"\x80", 0),
    (0x8C, b"\x01", 0), (0x8D, b"\x01", 0), (0x8E, b"\xFF", 0),
    (0x8F, b"\xFF", 0), (0xB6, b"\x00\x00", 0),
    (0x36, bytes([MADCTL]), 0), (0x3A, b"\x05", 0),
    (0x90, b"\x08\x08\x08\x08", 0), (0xBD, b"\x06", 0), (0xBC, b"\x00", 0),
    (0xFF, b"\x60\x01\x04", 0), (0xC3, b"\x13", 0), (0xC4, b"\x13", 0),
    (0xC9, b"\x22", 0), (0xBE, b"\x11", 0), (0xE1, b"\x10\x0E", 0),
    (0xDF, b"\x21\x0C\x02", 0),
    (0xF0, b"\x45\x09\x08\x08\x26\x2A", 0),
    (0xF1, b"\x43\x70\x72\x36\x37\x6F", 0),
    (0xF2, b"\x45\x09\x08\x08\x26\x2A", 0),
    (0xF3, b"\x43\x70\x72\x36\x37\x6F", 0),
    (0xED, b"\x1B\x0B", 0), (0xAE, b"\x77", 0), (0xCD, b"\x63", 0),
    (0x70, b"\x07\x07\x04\x0E\x0F\x09\x07\x08\x03", 0),
    (0xE8, b"\x34", 0),
    (0x62, b"\x18\x0D\x71\xED\x70\x70\x18\x0F\x71\xEF\x70\x70", 0),
    (0x63, b"\x18\x11\x71\xF1\x70\x70\x18\x13\x71\xF3\x70\x70", 0),
    (0x64, b"\x28\x29\xF1\x01\xF1\x00\x07", 0),
    (0x66, b"\x3C\x00\xCD\x67\x45\x45\x10\x00\x00\x00", 0),
    (0x67, b"\x00\x3C\x00\x00\x00\x01\x54\x10\x32\x98", 0),
    (0x74, b"\x10\x85\x80\x00\x00\x4E\x00", 0),
    (0x98, b"\x3E\x07", 0),
    (0x35, b"", 0),
    (0x21, b"", 0),      # display inversion on (IPS panel)
    (0x11, b"", 120),    # sleep out
    (0x29, b"", 20),     # display on
)


class GC9A01:
    def __init__(self):
        self.spi = SPI(1, baudrate=40_000_000, polarity=0, phase=0,
                       sck=Pin(LCD_SCK), mosi=Pin(LCD_MOSI))
        self.dc = Pin(LCD_DC, Pin.OUT, value=0)
        self.cs = Pin(LCD_CS, Pin.OUT, value=1)
        self.rst = Pin(LCD_RST, Pin.OUT, value=1)
        self.rst(0)
        time.sleep_ms(20)
        self.rst(1)
        time.sleep_ms(120)
        for cmd, data, delay in _INIT:
            self._cmd(cmd, data)
            if delay:
                time.sleep_ms(delay)

    def _cmd(self, c, data=b""):
        self.cs(0)
        self.dc(0)
        self.spi.write(bytes([c]))
        if data:
            self.dc(1)
            self.spi.write(data)
        self.cs(1)

    def flush(self, buf):
        self._cmd(0x2A, b"\x00\x00\x00\xEF")
        self._cmd(0x2B, b"\x00\x00\x00\xEF")
        self.cs(0)
        self.dc(0)
        self.spi.write(b"\x2C")
        self.dc(1)
        self.spi.write(buf)
        self.cs(1)


# =====================================================================
#  colors (RGB565, bytes pre-swapped so the buffer is sent as-is)
# =====================================================================
def rgb(r, g, b):
    c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return ((c & 0xFF) << 8) | (c >> 8)


BLACK_C = rgb(0, 0, 0)
WHITE_C = rgb(255, 255, 255)
PINK = rgb(255, 143, 177)
MINT = rgb(127, 224, 194)
YELLOW = rgb(255, 217, 90)
ALERT = rgb(200, 0, 0)
GRAY = rgb(110, 110, 110)
GHOST = rgb(30, 30, 30)    # unlit segments

# themes: name, main (HH:MM), seconds, date/accent
THEMES = (
    ("BUNNY", rgb(255, 255, 255), rgb(255, 143, 177), rgb(127, 224, 194)),
    ("ICE", rgb(190, 235, 255), rgb(0, 150, 255), rgb(120, 200, 255)),
    ("AMBER", rgb(255, 191, 0), rgb(255, 120, 0), rgb(255, 214, 120)),
    ("MATRIX", rgb(120, 255, 120), rgb(0, 200, 60), rgb(0, 170, 60)),
    ("SUNSET", rgb(255, 160, 120), rgb(255, 80, 120), rgb(255, 200, 150)),
    ("RED", rgb(255, 40, 40), rgb(170, 0, 0), rgb(210, 70, 70)),
)
C_MAIN, C_SEC, C_DATE = THEMES[0][1], THEMES[0][2], THEMES[0][3]

# user settings (saved in /settings.json)
STYLES = ("CLASSIC", "BIG", "ANALOG")
FMTS = ("24 HOUR", "12 HOUR")
BRIGHT_LEVELS = (100, 70, 40, 15)
ONOFF = ("OFF", "ON")
st_style = 0
st_theme = 0
st_h24 = 1 if USE_24H else 0
st_bright = 0
st_night = 0
st_rot = 0
st_flip = 0
al_on = 0
al_h = 7
al_m = 0
SET_FILE = "/settings.json"

buf = None
fb = None
lcd = None
bl = None
cur_pct = -1
wake_until = 0
wx_temp = None
wx_code = 0
wx_day = 1
wx_time = ""
next_wx = 0
rot = 0
imu_ok = False
_tri = array("h", [0, 0, 0, 0, 0, 0])
_cb = bytearray(8)
_cf = framebuf.FrameBuffer(_cb, 8, 8, framebuf.MONO_HLSB)

# =====================================================================
#  drawing helpers
# =====================================================================
SEG_MAP = ("abcdef", "bc", "abged", "abgcd", "fgbc",
           "afgcd", "afgedc", "abc", "abcdefg", "abcdfg")


def triangle(x1, y1, x2, y2, x3, y3, c):
    _tri[0] = x1; _tri[1] = y1
    _tri[2] = x2; _tri[3] = y2
    _tri[4] = x3; _tri[5] = y3
    fb.poly(0, 0, _tri, c, True)


def hseg(x1, x2, y, c, H):
    fb.fill_rect(x1 + H, y - H, x2 - x1 - 2 * H + 1, 2 * H + 1, c)
    triangle(x1, y, x1 + H, y - H, x1 + H, y + H, c)
    triangle(x2, y, x2 - H, y - H, x2 - H, y + H, c)


def vseg(x, y1, y2, c, H):
    fb.fill_rect(x - H, y1 + H, 2 * H + 1, y2 - y1 - 2 * H + 1, c)
    triangle(x, y1, x - H, y1 + H, x + H, y1 + H, c)
    triangle(x, y2, x - H, y2 - H, x + H, y2 - H, c)


def draw_digit(d, ox, oy, w, h, H, lit):
    g = H - 1
    m = oy + h // 2
    on = SEG_MAP[d]
    hseg(ox + g, ox + w - g, oy, lit if "a" in on else GHOST, H)
    vseg(ox + w, oy + g, m - g, lit if "b" in on else GHOST, H)
    vseg(ox + w, m + g, oy + h - g, lit if "c" in on else GHOST, H)
    hseg(ox + g, ox + w - g, oy + h, lit if "d" in on else GHOST, H)
    vseg(ox, m + g, oy + h - g, lit if "e" in on else GHOST, H)
    vseg(ox, oy + g, m - g, lit if "f" in on else GHOST, H)
    hseg(ox + g, ox + w - g, m, lit if "g" in on else GHOST, H)


BIG_W, BIG_H, BIG_T = 32, 64, 4
BIG_X = (30, 72, 136, 178)          # H H : M M
SM_W, SM_H, SM_T = 22, 40, 2
# digit geometry: (width, height, half-thickness, x positions, colon y offsets)
GEOM_CLASSIC = (BIG_W, BIG_H, BIG_T, BIG_X, 22, 46)
GEOM_BIG = (36, 76, 5, (24, 70, 134, 180), 26, 50)


def draw_big_time(a, b, oy, color, colon_on, hide_lead0=False, g=GEOM_CLASSIC):
    """a = hours or minutes (left pair), b = minutes or seconds (right pair)."""
    w, h, t, xs, c1, c2 = g
    d = (a // 10, a % 10, b // 10, b % 10)
    for i in range(4):
        if i == 0 and hide_lead0 and d[0] == 0:
            continue
        draw_digit(d[i], xs[i], oy, w, h, t, color)
    cc = color if colon_on else GHOST
    fb.fill_rect(116, oy + c1, 8, 8, cc)
    fb.fill_rect(116, oy + c2, 8, 8, cc)


def big_text(s, x, y, sc, color):
    """Draw text scaled up by sc using the built-in 8x8 font."""
    for ch in s:
        _cf.fill(0)
        _cf.text(ch, 0, 0, 1)
        for j in range(8):
            row = _cb[j]
            if row:
                for i in range(8):
                    if row & (0x80 >> i):
                        fb.fill_rect(x + i * sc, y + j * sc, sc, sc, color)
        x += 8 * sc


def big_text_c(s, y, sc, color):
    big_text(s, (240 - 8 * sc * len(s)) // 2, y, sc, color)


def text_c(s, y, color):
    fb.text(s, (240 - 8 * len(s)) // 2, y, color)


def splash(line1, line2=""):
    fb.fill(BLACK_C)
    big_text_c(line1, 90, 2, MINT)
    if line2:
        text_c(line2, 130, WHITE_C)
    lcd.flush(buf)


# =====================================================================
#  time: WiFi + NTP + timezone (with daylight saving)
# =====================================================================
DAYS = ("MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN")
MONTHS = ("JAN", "FEB", "MAR", "APR", "MAY", "JUN",
          "JUL", "AUG", "SEP", "OCT", "NOV", "DEC")

time_is_utc = False      # True after a successful NTP sync (RTC then holds UTC)
next_sync = 0


def nth_sunday(year, month, n):
    first_wd = time.localtime(time.mktime((year, month, 1, 12, 0, 0, 0, 0)))[6]
    first_sun = 1 + (6 - first_wd) % 7
    return first_sun + 7 * (n - 1)


def is_dst_utc(secs):
    y = time.localtime(secs)[0]
    start = time.mktime((y, 3, nth_sunday(y, 3, 2), 2, 0, 0, 0, 0)) - TZ_STD_OFFSET * 3600
    end = time.mktime((y, 11, nth_sunday(y, 11, 1), 2, 0, 0, 0, 0)) - (TZ_STD_OFFSET + 1) * 3600
    return start <= secs < end


def local_now():
    secs = time.time()
    if time_is_utc:
        off = TZ_STD_OFFSET
        if USE_DST and is_dst_utc(secs):
            off += 1
        secs += off * 3600
    else:
        secs += int(MANUAL_OFFSET_HOURS * 3600)
    return time.localtime(secs)


CRED_FILE = "/wifi.json"
wifi_ssid = WIFI_SSID
wifi_pass = WIFI_PASSWORD
wifi_problem = False     # True when the last sync failed because WiFi could not connect
sync_fails = 0


def load_creds():
    global wifi_ssid, wifi_pass
    try:
        import json
        with open(CRED_FILE) as f:
            d = json.load(f)
        if d.get("ssid"):
            wifi_ssid = d["ssid"]
            wifi_pass = d.get("password", "")
    except Exception:
        pass


def save_creds(ssid, pw):
    import json
    with open(CRED_FILE, "w") as f:
        json.dump({"ssid": ssid, "password": pw}, f)


def wifi_connect(timeout_s=10):
    import network
    wlan = network.WLAN(network.STA_IF)
    for attempt in range(2):
        wlan.active(True)
        if wlan.isconnected():
            return wlan
        try:
            wlan.connect(wifi_ssid, wifi_pass)
        except OSError as e:
            print("wifi connect error:", e)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_s * 1000:
                break
            time.sleep_ms(200)
        if wlan.isconnected():
            return wlan
        try:
            wlan.disconnect()
        except Exception:
            pass
        wlan.active(False)
        time.sleep_ms(500)
    return None


def ntp_sync():
    """Returns True if the clock was set from the internet."""
    global time_is_utc, wifi_problem
    wifi_problem = False
    if wifi_ssid == "YOUR_WIFI_NAME":
        print("WiFi not configured")
        wifi_problem = True
        return False
    ok = False
    try:
        wlan = wifi_connect()
        if wlan is None:
            print("WiFi connect failed")
            wifi_problem = True
            return False
        import ntptime
        ntptime.timeout = 5
        for _ in range(3):
            try:
                ntptime.settime()      # sets the RTC to UTC
                ok = True
                break
            except OSError:
                time.sleep_ms(500)
        try:
            wlan.disconnect()
            wlan.active(False)
        except Exception:
            pass
    except Exception as e:
        print("NTP error:", e)
    if ok:
        time_is_utc = True
        print("NTP sync OK")
    else:
        print("NTP sync failed")
    return ok


def schedule_sync(ok):
    global next_sync
    ms = RESYNC_HOURS * 3600 * 1000 if ok else 10 * 60 * 1000
    next_sync = time.ticks_add(time.ticks_ms(), ms)


# =====================================================================
#  settings (theme, style, format, brightness) + backlight
# =====================================================================
def apply_theme():
    global C_MAIN, C_SEC, C_DATE
    th = THEMES[st_theme % len(THEMES)]
    C_MAIN, C_SEC, C_DATE = th[1], th[2], th[3]


def load_settings():
    global st_style, st_theme, st_h24, st_bright, st_night, st_rot, st_flip, al_on, al_h, al_m
    try:
        import json
        with open(SET_FILE) as f:
            d = json.load(f)
        st_style = int(d.get("style", st_style)) % len(STYLES)
        st_theme = int(d.get("theme", st_theme)) % len(THEMES)
        st_h24 = int(d.get("h24", st_h24)) % 2
        st_bright = int(d.get("bright", st_bright)) % len(BRIGHT_LEVELS)
        st_night = int(d.get("night", st_night)) % 2
        st_rot = int(d.get("rot", st_rot)) % 2
        st_flip = int(d.get("flip", st_flip)) % 2
        al_on = int(d.get("al_on", al_on)) % 2
        al_h = int(d.get("al_h", al_h)) % 24
        al_m = int(d.get("al_m", al_m)) % 60
    except Exception:
        pass
    apply_theme()


def save_settings():
    try:
        import json
        with open(SET_FILE, "w") as f:
            json.dump({"style": st_style, "theme": st_theme, "h24": st_h24,
                       "bright": st_bright, "night": st_night,
                       "rot": st_rot, "flip": st_flip,
                       "al_on": al_on, "al_h": al_h, "al_m": al_m}, f)
    except Exception as e:
        print("settings save error:", e)


def set_backlight(pct):
    global cur_pct
    if bl is not None and pct != cur_pct:
        cur_pct = pct
        bl.duty_u16(65535 * pct // 100)


def want_brightness(now, lt):
    pct = BRIGHT_LEVELS[st_bright]
    if al_ring:
        return 100
    if st_flip and _flip_down and timer_state == T_RUN:
        return 0
    if batt_low:
        pct = min(pct, 15)
    if st_night and lt[0] >= 2024 and (lt[3] >= NIGHT_START or lt[3] < NIGHT_END):
        if time.ticks_diff(wake_until, now) < 0:
            pct = min(pct, NIGHT_PCT)
    return pct


# =====================================================================
#  weather (Open-Meteo, free, no key) and motion sensor (QMI8658)
# =====================================================================
def wx_text(c):
    if c == 0:
        return "CLEAR"
    if c == 1:
        return "FAIR"
    if c == 2:
        return "PT CLOUDY"
    if c == 3:
        return "OVERCAST"
    if c in (45, 48):
        return "FOG"
    if c in (51, 53, 55):
        return "DRIZZLE"
    if c in (56, 57):
        return "FRZ DRIZZLE"
    if c in (61, 63, 65):
        return "RAIN"
    if c in (66, 67):
        return "FRZ RAIN"
    if c in (71, 73, 75):
        return "SNOW"
    if c == 77:
        return "SNOW GRAINS"
    if c in (80, 81, 82):
        return "SHOWERS"
    if c in (85, 86):
        return "SNOW SHWRS"
    if c >= 95:
        return "STORM"
    return "WEATHER"


def weather_sync():
    """Fetch the current weather. Returns True on success."""
    global wx_temp, wx_code, wx_day, wx_time
    if wifi_ssid == "YOUR_WIFI_NAME":
        return False
    ok = False
    try:
        wlan = wifi_connect()
        if wlan is None:
            print("weather: WiFi failed")
            return False
        import requests
        gc.collect()
        url = ("https://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s"
               "&current=temperature_2m,weather_code,is_day&timezone=auto"
               % (WEATHER_LAT, WEATHER_LON))
        r = None
        for attempt in range(3):
            try:
                r = requests.get(url)
                break
            except Exception as e:
                print("weather retry:", e)
                gc.collect()
                time.sleep_ms(1500)
        if r is None:
            raise OSError("no reply")
        try:
            c = r.json()["current"]
            wx_temp = c["temperature_2m"]
            wx_code = int(c["weather_code"])
            wx_day = int(c.get("is_day", 1))
            lt = local_now()
            wx_time = "%02d:%02d" % (lt[3], lt[4])
            ok = True
        finally:
            r.close()
        try:
            wlan.disconnect()
            wlan.active(False)
        except Exception:
            pass
    except Exception as e:
        print("weather error:", e)
    print("weather OK" if ok else "weather failed")
    return ok


def schedule_wx(ok):
    global next_wx
    ms = WEATHER_MIN * 60 * 1000 if ok else 10 * 60 * 1000
    next_wx = time.ticks_add(time.ticks_ms(), ms)


IMU_ADDR = 0x6B
_imu_t = 0
_imu_prev = (0, 0, 0)
_shake_armed = False
_shake_t = 0
_rot_cand = 0
_rot_t = 0


def imu_init():
    global imu_ok
    imu_ok = False
    try:
        if IMU_ADDR not in i2c.scan():
            print("IMU not found")
            return
        if i2c.readfrom_mem(IMU_ADDR, 0x00, 1)[0] != 0x05:
            print("IMU: unexpected chip id")
            return
        i2c.writeto_mem(IMU_ADDR, 0x02, b"\x40")   # register auto-increment
        i2c.writeto_mem(IMU_ADDR, 0x03, b"\x16")   # accelerometer 4 g, 125 Hz
        i2c.writeto_mem(IMU_ADDR, 0x08, b"\x01")   # accelerometer on
        imu_ok = True
    except Exception as e:
        print("IMU error:", e)
    print("IMU ready:", imu_ok)


def set_rotation(r):
    global rot, dirty
    rot = r
    if lcd is not None:
        lcd._cmd(0x36, bytes([MADCTL ^ (0xC0 if r else 0)]))
    dirty = True


def imu_poll(now):
    """Returns (shake, moved). Also turns the screen when AUTO ROTATE is on."""
    global _imu_t, _imu_prev, _shake_armed, _shake_t, _rot_cand, _rot_t
    if not imu_ok or time.ticks_diff(now, _imu_t) < 40:
        return False, False
    _imu_t = now
    try:
        ax, ay, az = struct.unpack("<hhh", i2c.readfrom_mem(IMU_ADDR, 0x35, 6))
    except Exception:
        return False, False
    px, py, pz = _imu_prev
    _imu_prev = (ax, ay, az)
    moved = (abs(ax - px) + abs(ay - py) + abs(az - pz)) > MOTION_G * 8192
    shake = False
    if (ax * ax + ay * ay + az * az) > (SHAKE_G * 8192) ** 2:
        if _shake_armed and time.ticks_diff(now, _shake_t) < 800:
            shake = True
            _shake_armed = False
        else:
            _shake_armed = True
            _shake_t = now
    if st_rot:
        want = rot
        v = ay * IMU_UP_SIGN
        if v > 4500:
            want = 0
        elif v < -4500:
            want = 1
        if want != rot:
            if want == _rot_cand:
                if time.ticks_diff(now, _rot_t) > 1000:
                    set_rotation(want)
            else:
                _rot_cand = want
                _rot_t = now
        else:
            _rot_cand = rot
    return shake, moved


def wake(now):
    global wake_until
    wake_until = time.ticks_add(now, WAKE_MS)


def shake_event():
    global timer_state, dirty
    if al_ring:
        alarm_snooze()
        return
    if screen == SCR_TIMER and timer_state == T_DONE:
        timer_state = T_SET
        dirty = True


# =====================================================================
#  touch + button
# =====================================================================
i2c = None
touch_found = False
button = None

HOLD_MS = 700
REPEAT_START_MS = 500
REPEAT_EVERY_MS = 150
pressed = False
press_t = 0
press_zone = 1
long_fired = False
last_rep = 0


def setup_touch():
    global i2c, touch_found
    rst = Pin(TP_RST, Pin.OUT)
    rst(0)
    time.sleep_ms(10)
    rst(1)
    time.sleep_ms(50)
    i2c = I2C(0, sda=Pin(TP_SDA), scl=Pin(TP_SCL), freq=400000)
    touch_found = TOUCH_ADDR in i2c.scan()
    if touch_found:
        try:
            i2c.writeto_mem(TOUCH_ADDR, 0xFE, b"\x01")   # keep the chip awake
        except OSError:
            pass
    print("Touch chip found:", touch_found)


def read_touch():
    """Returns (touched, x)."""
    if not touch_found:
        return False, 0
    try:
        d = i2c.readfrom_mem(TOUCH_ADDR, 0x02, 3)
    except OSError:
        return False, 0
    if d[0] == 0:
        return False, 0
    return True, ((d[1] & 0x0F) << 8) | d[2]


def zone_of(x):
    if TOUCH_FLIP_X != (rot == 1):
        x = 239 - x
    return 0 if x < 80 else 2 if x > 160 else 1     # 0 = left, 1 = middle, 2 = right


def poll_input(now):
    """Returns ('down'|'tap'|'long', zone) or None."""
    global pressed, press_t, press_zone, long_fired
    t, x = read_touch()
    b = button.value() == 0
    touched = t or b
    zone = 1 if (b and not t) else zone_of(x)
    ev = None
    if touched and not pressed:
        pressed = True
        press_t = now
        press_zone = zone
        long_fired = False
        ev = ("down", zone)
    elif touched and pressed:
        if not long_fired and time.ticks_diff(now, press_t) >= HOLD_MS:
            long_fired = True
            ev = ("long", press_zone)
    elif pressed:
        pressed = False
        if not long_fired:
            ev = ("tap", press_zone)
    return ev


# =====================================================================
#  timer logic
# =====================================================================
SCR_CLOCK, SCR_TIMER, SCR_SET, SCR_WX, SCR_WORLD = 0, 1, 2, 3, 4
SCR_ASK = 5
SCR_BATT, SCR_ALARM = 6, 7
set_sel = 0
ITEMS = ("STYLE", "THEME", "FORMAT", "BRIGHTNESS", "NIGHT DIM", "AUTO ROTATE", "FLIP TIMER")
T_SET, T_RUN, T_PAUSE, T_DONE = 0, 1, 2, 3

screen = SCR_CLOCK
timer_state = T_SET
timer_min = DEFAULT_TIMER_MIN
timer_left_ms = 0
timer_end = 0
done_t = 0
dirty = True


def timer_secs():
    if timer_state == T_SET:
        return timer_min * 60
    if timer_state == T_DONE:
        return 0
    return (timer_left_ms + 999) // 1000


def step_minutes(zone, now):
    global timer_min, last_rep
    timer_min += -1 if zone == 0 else 1
    if timer_min < 1:
        timer_min = 1
    if timer_min > 99:
        timer_min = 99
    last_rep = now


def timer_start(now):
    global timer_state, timer_left_ms, timer_end
    timer_left_ms = timer_min * 60000
    timer_end = time.ticks_add(now, timer_left_ms)
    timer_state = T_RUN


def timer_toggle_pause(now):
    global timer_state, timer_end
    if timer_state == T_RUN:
        timer_state = T_PAUSE
    elif timer_state == T_PAUSE:
        timer_end = time.ticks_add(now, timer_left_ms)
        timer_state = T_RUN


def update_timer(now):
    global timer_state, timer_left_ms, screen, done_t, dirty
    if timer_state == T_RUN:
        timer_left_ms = time.ticks_diff(timer_end, now)
        if timer_left_ms <= 0:
            timer_left_ms = 0
            timer_state = T_DONE
            screen = SCR_TIMER
            done_t = now
            dirty = True
    elif timer_state == T_DONE:
        if time.ticks_diff(now, done_t) > 60000:     # stop flashing after 1 minute
            timer_state = T_SET
            dirty = True


def handle_event(ev, now):
    global screen, timer_state, dirty, wake_until, al_f
    kind, zone = ev
    dirty = True
    wake_until = time.ticks_add(now, WAKE_MS)
    if al_ring:
        if kind != "down":
            alarm_stop()
        return
    if screen == SCR_CLOCK:
        if kind == "tap":
            if zone == 1:
                screen = SCR_TIMER
            elif zone == 2:
                screen = SCR_WX
            else:
                screen = SCR_ALARM
                al_f = 0
        elif kind == "long":
            screen = SCR_SET
        return
    if screen == SCR_WX or screen == SCR_WORLD or screen == SCR_BATT:
        if kind == "tap":
            if zone == 1:
                screen = SCR_CLOCK
            elif zone == 2:
                screen = (SCR_WORLD if screen == SCR_WX else
                          SCR_BATT if screen == SCR_WORLD else SCR_ASK)
            else:
                screen = (SCR_CLOCK if screen == SCR_WX else
                          SCR_WX if screen == SCR_WORLD else SCR_WORLD)
        elif kind == "long":
            screen = SCR_CLOCK
        return
    if screen == SCR_SET:
        settings_event(kind, zone)
        return
    if screen == SCR_ASK:
        ask_event(kind, zone, now)
        return
    if screen == SCR_ALARM:
        alarm_event(kind, zone, now)
        return

    if timer_state == T_SET:
        if kind == "down" and zone != 1:
            step_minutes(zone, now)
        elif kind == "tap" and zone == 1:
            timer_start(now)
        elif kind == "long" and zone == 1:
            screen = SCR_CLOCK
    elif timer_state in (T_RUN, T_PAUSE):
        if kind == "tap":
            if zone == 1:
                timer_toggle_pause(now)
            else:
                screen = SCR_CLOCK
        elif kind == "long":
            timer_state = T_SET
    else:   # T_DONE
        if kind == "tap":
            timer_state = T_SET
        elif kind == "long":
            timer_state = T_SET
            screen = SCR_CLOCK


def settings_event(kind, zone):
    global screen, set_sel, st_style, st_theme, st_h24, st_bright, st_night, st_rot, st_flip
    if kind == "long":
        save_settings()
        screen = SCR_CLOCK
    elif kind == "tap":
        if zone == 0:
            set_sel = (set_sel - 1) % len(ITEMS)
        elif zone == 2:
            set_sel = (set_sel + 1) % len(ITEMS)
        elif set_sel == 0:
            st_style = (st_style + 1) % len(STYLES)
        elif set_sel == 1:
            st_theme = (st_theme + 1) % len(THEMES)
            apply_theme()
        elif set_sel == 2:
            st_h24 = 1 - st_h24
        elif set_sel == 3:
            st_bright = (st_bright + 1) % len(BRIGHT_LEVELS)
            set_backlight(BRIGHT_LEVELS[st_bright])
        elif set_sel == 4:
            st_night = 1 - st_night
        elif set_sel == 5:
            st_rot = 1 - st_rot
            if not st_rot:
                set_rotation(0)
        else:
            st_flip = 1 - st_flip


# =====================================================================
#  screens
# =====================================================================
def timer_reminder(y):
    if timer_state in (T_RUN, T_PAUSE):
        sc = timer_secs()
        text_c("TIMER %02d:%02d" % (sc // 60, sc % 60), y, C_SEC)


def hint_text(sec):
    p = (sec // 4) % 5
    if p == 4 and al_on:
        hh, sf, _h0 = fmt_hour(al_h)
        return "ALARM %d:%02d%s" % (hh, al_m, sf)
    if p == 0:
        return "TAP = TIMER"
    if p == 1:
        return "SIDES = MORE"
    if p == 2 and wx_temp is not None:
        return "%d C  %s" % (round(wx_temp), wx_text(wx_code))
    return "HOLD = SETTINGS"


def fmt_hour(hour):
    if st_h24:
        return hour, "", False
    return hour % 12 or 12, (" PM" if hour >= 12 else " AM"), True


def date_line(lt, suffix):
    year, month, day, wd = lt[0], lt[1], lt[2], lt[6]
    return "%s %d %s %d%s" % (DAYS[wd], day, MONTHS[month - 1], year, suffix)


def draw_classic(lt):
    hour, suffix, hide0 = fmt_hour(lt[3])
    sec = lt[5]
    fb.fill(BLACK_C)
    timer_reminder(26)
    draw_big_time(hour, lt[4], 54, C_MAIN, sec % 2 == 0, hide0)
    draw_digit(sec // 10, 94, 140, SM_W, SM_H, SM_T, C_SEC)
    draw_digit(sec % 10, 124, 140, SM_W, SM_H, SM_T, C_SEC)
    if lt[0] < 2024:
        text_c("TIME NOT SET", 196, YELLOW)
    else:
        text_c(date_line(lt, suffix), 196, C_DATE)
    text_c(hint_text(sec), 214, GRAY)


def draw_bigstyle(lt):
    hour, suffix, hide0 = fmt_hour(lt[3])
    sec = lt[5]
    fb.fill(BLACK_C)
    timer_reminder(38)
    draw_big_time(hour, lt[4], 62, C_MAIN, sec % 2 == 0, hide0, GEOM_BIG)
    # seconds progress bar
    fb.fill_rect(50, 164, 140, 6, GHOST)
    fb.fill_rect(50, 164, 140 * (sec + 1) // 60, 6, C_SEC)
    if lt[0] < 2024:
        text_c("TIME NOT SET", 184, YELLOW)
    else:
        text_c(date_line(lt, suffix), 184, C_DATE)
    text_c(hint_text(sec), 204, GRAY)


_q = array("h", [0, 0, 0, 0, 0, 0, 0, 0])


def _pt(r, ang):
    a = radians(ang)
    return int(120 + r * sin(a) + 0.5), int(120 - r * cos(a) + 0.5)


def _hand(ang, length, width, tail, color):
    """Kite shaped hand pointing at ang (degrees, clockwise from 12)."""
    tx, ty = _pt(length, ang)
    bx, by = _pt(-tail, ang)
    a = radians(ang + 90)
    ox = int(width * sin(a) + (0.5 if sin(a) >= 0 else -0.5))
    oy = int(-width * cos(a) + (0.5 if cos(a) <= 0 else -0.5))
    _q[0] = 120 + ox; _q[1] = 120 + oy
    _q[2] = tx;       _q[3] = ty
    _q[4] = 120 - ox; _q[5] = 120 - oy
    _q[6] = bx;       _q[7] = by
    fb.poly(0, 0, _q, color, True)


def draw_analog(lt):
    hour, minute, sec = lt[3], lt[4], lt[5]
    fb.fill(BLACK_C)
    # minute ticks and hour ticks
    for i in range(60):
        ang = i * 6
        if i % 5 == 0:
            _q[0], _q[1] = _pt(100, ang - 1.4)
            _q[2], _q[3] = _pt(100, ang + 1.4)
            _q[4], _q[5] = _pt(117, ang + 1.4)
            _q[6], _q[7] = _pt(117, ang - 1.4)
            fb.poly(0, 0, _q, C_MAIN, True)
        else:
            x, y = _pt(114, ang)
            fb.fill_rect(x - 1, y - 1, 2, 2, GRAY)
    # numbers at 12 / 3 / 6 / 9
    big_text_c("12", 70, 1, C_DATE)
    big_text("3", 168, 116, 1, C_DATE)
    big_text("6", 116, 162, 1, C_DATE)
    big_text("9", 66, 116, 1, C_DATE)
    if lt[0] >= 2024:
        text_c("%s %d" % (DAYS[lt[6]], lt[2]), 140, C_DATE)
    timer_reminder(96)
    _hand((hour % 12) * 30 + minute * 0.5, 58, 5, 10, C_MAIN)
    _hand(minute * 6 + sec * 0.1, 88, 4, 12, C_MAIN)
    # second hand
    x1, y1 = _pt(100, sec * 6)
    x0, y0 = _pt(-20, sec * 6)
    fb.line(x0, y0, x1, y1, C_SEC)
    fb.fill_rect(117, 117, 7, 7, C_SEC)


def draw_clock_screen(lt):
    if st_style == 2:
        draw_analog(lt)
    elif st_style == 1:
        draw_bigstyle(lt)
    else:
        draw_classic(lt)


LIGHT = rgb(190, 200, 215)
DARKC = rgb(130, 140, 160)
SUNC = rgb(255, 200, 40)
MOONC = rgb(235, 235, 190)
BLUE = rgb(80, 160, 255)
_bz = array("h", [0] * 12)


def _sun(cx, cy, r):
    fb.ellipse(cx, cy, r, r, SUNC, True)
    for i in range(8):
        a = radians(i * 45)
        s, c = sin(a), cos(a)
        fb.line(cx + int((r + 5) * s), cy - int((r + 5) * c),
                cx + int((r + 11) * s), cy - int((r + 11) * c), SUNC)


def _moon(cx, cy, r):
    fb.ellipse(cx, cy, r, r, MOONC, True)
    fb.ellipse(cx + r // 2, cy - r // 3, r - 2, r - 2, BLACK_C, True)


def _cloud(cx, cy, c):
    fb.ellipse(cx - 13, cy + 4, 12, 10, c, True)
    fb.ellipse(cx + 13, cy + 5, 12, 9, c, True)
    fb.ellipse(cx, cy - 4, 15, 13, c, True)
    fb.fill_rect(cx - 13, cy + 4, 27, 10, c)


def draw_wx_icon(cx, cy, code, day):
    if code <= 1:
        if day:
            _sun(cx, cy, 13)
        else:
            _moon(cx, cy, 16)
    elif code == 2:
        if day:
            _sun(cx + 12, cy - 8, 9)
        else:
            _moon(cx + 12, cy - 8, 11)
        _cloud(cx - 4, cy + 6, LIGHT)
    elif code in (51, 53, 55, 56, 57, 61, 63, 65, 66, 67, 80, 81, 82):
        _cloud(cx, cy - 4, DARKC)
        for i in (-12, 0, 12):
            fb.line(cx + i, cy + 14, cx + i - 4, cy + 25, BLUE)
            fb.line(cx + i + 1, cy + 14, cx + i - 3, cy + 25, BLUE)
    elif code in (71, 73, 75, 77, 85, 86):
        _cloud(cx, cy - 4, LIGHT)
        for k, i in enumerate((-12, 0, 12)):
            y = cy + 17 + (k % 2) * 6
            fb.fill_rect(cx + i - 1, y, 3, 3, WHITE_C)
            fb.fill_rect(cx + i - 1, y + 9, 3, 3, WHITE_C)
    elif code >= 95:
        _cloud(cx, cy - 6, DARKC)
        pts = (4, 8, -6, 21, 0, 21, -4, 33, 9, 17, 3, 17)
        for i in range(12):
            _bz[i] = (cx if i % 2 == 0 else cy) + pts[i]
        fb.poly(0, 0, _bz, YELLOW, True)
    else:
        _cloud(cx, cy, LIGHT)
        if code in (45, 48):
            for y in (cy + 18, cy + 24, cy + 30):
                fb.hline(cx - 20, y, 40, GRAY)


def draw_weather_screen():
    fb.fill(BLACK_C)
    big_text_c(WEATHER_CITY, 22, 2, C_SEC)
    if wx_temp is None:
        big_text_c("NO DATA", 96, 2, YELLOW)
        text_c("WAITING FOR WIFI", 124, GRAY)
    else:
        draw_wx_icon(120, 78, wx_code, wx_day)
        s = str(round(wx_temp))
        sc = 4
        dw = 8 * sc * len(s)
        x0 = (240 - (dw + 11 + 8 * sc)) // 2
        big_text(s, x0, 112, sc, C_MAIN)
        fb.ellipse(x0 + dw + 5, 118, 4, 4, C_MAIN)
        big_text("C", x0 + dw + 11, 112, sc, C_MAIN)
        txt = wx_text(wx_code)
        big_text_c(txt, 156, 2 if len(txt) <= 11 else 1, C_DATE)
        if wx_time:
            text_c("UPDATED " + wx_time, 184, GRAY)
    text_c("SIDES = NEXT PAGE", 198, GRAY)
    text_c("MIDDLE = CLOCK", 210, GRAY)


def draw_world_screen(lt):
    fb.fill(BLACK_C)
    big_text_c(WORLD_CITY, 24, 2, C_SEC)
    if not time_is_utc:
        big_text_c("NO SYNC", 96, 2, YELLOW)
        text_c("WAITING FOR WIFI TIME", 124, GRAY)
    else:
        w = time.localtime(time.time() + WORLD_OFFSET * 3600)
        hour, suffix, hide0 = fmt_hour(w[3])
        draw_big_time(hour, w[4], 50, C_MAIN, w[5] % 2 == 0, hide0)
        text_c(date_line(w, suffix), 124, C_DATE)
        lh, lsuf, _ = fmt_hour(lt[3])
        text_c("%s %02d:%02d%s" % (WEATHER_CITY, lh, lt[4], lsuf), 150, C_MAIN)
        diff = WORLD_OFFSET - (time.mktime(lt) - time.time()) // 3600
        if diff >= 0:
            text_c("+%d HOURS AHEAD" % diff, 166, GRAY)
        else:
            text_c("%d HOURS BEHIND" % -diff, 166, GRAY)
    text_c("SIDES = NEXT PAGE", 198, GRAY)
    text_c("MIDDLE = CLOCK", 210, GRAY)


def draw_settings_screen():
    fb.fill(BLACK_C)
    big_text_c("SETTINGS", 30, 2, C_MAIN)
    text_c("%d / %d" % (set_sel + 1, len(ITEMS)), 54, GRAY)
    big_text_c(ITEMS[set_sel], 68, 2, C_SEC)
    if set_sel == 0:
        val = STYLES[st_style]
    elif set_sel == 1:
        val = THEMES[st_theme][0]
    elif set_sel == 2:
        val = FMTS[0] if st_h24 else FMTS[1]
    elif set_sel == 3:
        val = "%d%%" % BRIGHT_LEVELS[st_bright]
    elif set_sel == 4:
        val = ONOFF[st_night]
    elif set_sel == 5:
        val = ONOFF[st_rot]
    else:
        val = ONOFF[st_flip]
    big_text_c(val, 104, 3, C_MAIN)
    big_text("<", 10, 104, 3, C_DATE)
    big_text(">", 206, 104, 3, C_DATE)
    if set_sel == 1:
        th = THEMES[st_theme]
        for i in range(3):
            fb.fill_rect(72 + i * 36, 140, 30, 12, th[1 + i])
    text_c("TAP MIDDLE = CHANGE", 166, WHITE_C)
    text_c("SIDES = PREV / NEXT", 178, GRAY)
    text_c("HOLD = DONE", 194, GRAY)


def draw_timer_screen(now):
    s = timer_state
    bg = ALERT if (s == T_DONE and (now // 400) % 2 == 0) else BLACK_C
    fb.fill(bg)

    secs = timer_secs()
    mm, ss = secs // 60, secs % 60

    if s == T_SET:
        title, color = "TIMER", C_DATE
    elif s == T_RUN:
        title, color = "TIMER", C_MAIN
    elif s == T_PAUSE:
        title, color = "PAUSED", C_SEC
        if (now // 500) % 2:
            color = GRAY
    else:
        title, color = "TIME UP", WHITE_C

    big_text_c(title, 22, 2, color)
    colon_on = True if s != T_RUN else (now // 500) % 2 == 0
    draw_big_time(mm, ss, 62, color, colon_on)

    if s == T_SET:
        big_text("-", 44, 150, 4, C_DATE)
        big_text("+", 164, 150, 4, C_DATE)
        text_c("TAP", 162, C_DATE)
        text_c("START", 174, C_DATE)
        if st_flip:
            text_c("OR FLIP FACE-DOWN", 188, MINT)
        text_c("HOLD = CLOCK", 206, GRAY)
    elif s in (T_RUN, T_PAUSE):
        text_c("TAP = PAUSE", 158, C_MAIN)
        text_c("SIDE = CLOCK", 172, GRAY)
        if st_flip:
            text_c("FLIP UP = PAUSE", 188, MINT)
        text_c("HOLD = STOP", 206, GRAY)
    else:
        text_c("TAP / SHAKE = CLOSE", 160, WHITE_C)
        text_c("HOLD = CLOCK", 206, WHITE_C)


# =====================================================================
#  WiFi setup (phone portal)
# =====================================================================
def scan_networks(sta):
    """Nearby networks, strongest first: [(ssid, rssi, secured), ...]"""
    best = {}
    try:
        for r in sta.scan():
            try:
                name = r[0].decode()
            except Exception:
                continue
            if not name:
                continue
            if name not in best or r[3] > best[name][0]:
                best[name] = (r[3], r[4] != 0)
    except Exception as e:
        print("scan error:", e)
    out = [(n, v[0], v[1]) for n, v in best.items()]
    out.sort(key=lambda t: -t[1])
    return out[:20]


def draw_setup_screen(nets, pw, ip):
    fb.fill(BLACK_C)
    big_text_c("WIFI SETUP", 34, 2, YELLOW)
    text_c("1. join this WiFi:", 58, PINK)
    big_text_c(SETUP_AP_NAME, 70, 2, MINT)
    text_c("password:", 92, PINK)
    big_text_c(pw, 104, 2, WHITE_C)
    text_c("2. open in browser:", 126, PINK)
    big_text_c(ip, 138, 2, MINT)
    text_c("networks nearby: %d" % len(nets), 162, GRAY)
    y = 174
    for n, rssi, sec in nets[:3]:
        text_c("%s %d" % (n[:15], rssi), y, WHITE_C)
        y += 11
    text_c("HOLD = CANCEL", 210, GRAY)
    lcd.flush(buf)


def setup_msg(line1, line2="", color=MINT):
    fb.fill(BLACK_C)
    big_text_c(line1[:13], 90, 2, color)
    if line2:
        text_c(line2[:24], 130, WHITE_C)
    lcd.flush(buf)


def _unquote(s):
    s = s.replace("+", " ")
    parts = s.split("%")
    out = bytearray(parts[0].encode())
    for p in parts[1:]:
        try:
            out.append(int(p[:2], 16))
            out += p[2:].encode()
        except Exception:
            out += b"%" + p.encode()
    return bytes(out).decode()


def _parse_form(body):
    out = {}
    for pair in body.decode().split("&"):
        if "=" in pair:
            k, v = pair.split("=", 1)
            out[_unquote(k)] = _unquote(v)
    return out


def _esc(t):
    return (t.replace("&", "&amp;").replace("<", "&lt;")
             .replace(">", "&gt;").replace('"', "&quot;"))


_CSS = ("body{font-family:sans-serif;margin:18px;background:#111;color:#eee}"
        "select,input,button{width:100%;font-size:18px;padding:10px;margin:6px 0;"
        "box-sizing:border-box}button{background:#7fe0c2;border:0;border-radius:6px}"
        "a{color:#7fe0c2}")


def _page(body):
    return ("<!DOCTYPE html><html><head><meta charset=utf-8>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>Clock WiFi</title><style>" + _CSS + "</style></head><body>"
            + body + "</body></html>")


def _form_page(nets, note=""):
    opts = ""
    for n, rssi, sec in nets:
        opts += "<option value=\"%s\">%s  (%d dBm)%s</option>" % (
            _esc(n), _esc(n), rssi, " *" if sec else "")
    return _page(
        "<h2>Clock WiFi setup</h2>" + note +
        "<form method=post action=/save>"
        "<label>Network (strongest first)</label><select name=ssid>" + opts + "</select>"
        "<label>or type a name (hidden network)</label><input name=manual>"
        "<label>Password</label><input name=password type=password>"
        "<button>Connect</button></form>"
        "<p><a href=/rescan>Scan again</a></p>")


def _read_request(c):
    c.settimeout(3)
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = c.recv(1024)
        if not chunk:
            break
        data += chunk
        if len(data) > 4096:
            break
    head, _, body = data.partition(b"\r\n\r\n")
    lines = head.split(b"\r\n")
    first = lines[0].decode().split(" ")
    n = 0
    for ln in lines[1:]:
        if ln.lower().startswith(b"content-length:"):
            n = int(ln.split(b":")[1])
    while len(body) < n and len(body) < 2048:
        chunk = c.recv(1024)
        if not chunk:
            break
        body += chunk
    return first[0], (first[1] if len(first) > 1 else "/"), body


def _send(c, html):
    c.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
              b"Connection: close\r\n\r\n")
    data = html.encode()
    for i in range(0, len(data), 1024):
        c.sendall(data[i:i + 1024])


def _try_connect(sta, ssid, pw):
    """Returns (ok, reason)."""
    import network
    sta.disconnect()
    sta.connect(ssid, pw)
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 20000:
        if sta.isconnected():
            return True, ""
        st = sta.status()
        if st == network.STAT_WRONG_PASSWORD:
            return False, "WRONG PASSWORD"
        if st == network.STAT_NO_AP_FOUND:
            return False, "NETWORK NOT FOUND"
        time.sleep_ms(300)
    return False, "NO CONNECTION"


def wifi_setup():
    """Phone-based WiFi setup. Returns True when new WiFi details were saved."""
    global dirty, wifi_ssid, wifi_pass
    import network
    import socket
    import os
    setup_msg("WIFI SETUP", "scanning networks...", YELLOW)
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    nets = scan_networks(sta)
    pw = "".join(str(b % 10) for b in os.urandom(8))
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(ssid=SETUP_AP_NAME, password=pw, authmode=network.AUTH_WPA2_PSK)
    ip = ap.ifconfig()[0]
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", 80))
    srv.listen(2)
    srv.settimeout(0.2)
    print("WiFi setup: join", SETUP_AP_NAME, "password", pw, "open", ip)
    draw_setup_screen(nets, pw, ip)

    saved = False
    t0 = time.ticks_ms()
    try:
        while time.ticks_diff(time.ticks_ms(), t0) < SETUP_TIMEOUT_MIN * 60000:
            ev = poll_input(time.ticks_ms())
            if ev and ev[0] == "long":
                break
            try:
                c, _ = srv.accept()
            except OSError:
                continue
            todo = None
            try:
                method, path, body = _read_request(c)
                if method == "POST" and path.startswith("/save"):
                    f = _parse_form(body)
                    name = (f.get("manual") or "").strip() or f.get("ssid", "")
                    if name:
                        _send(c, _page("<h2>Connecting to " + _esc(name) + "...</h2>"
                                       "<p>Watch the clock screen. If it says CONNECTED "
                                       "you are done. If not, reload this page "
                                       "(http://" + ip + ") and try again.</p>"))
                        todo = (name, f.get("password", ""))
                    else:
                        _send(c, _form_page(nets, "<p>Pick a network first.</p>"))
                elif path.startswith("/rescan"):
                    nets = scan_networks(sta)
                    draw_setup_screen(nets, pw, ip)
                    _send(c, _form_page(nets))
                else:
                    _send(c, _form_page(nets))
            except Exception as e:
                print("http error:", e)
            try:
                c.close()
            except Exception:
                pass
            if todo:
                setup_msg("CONNECTING", todo[0], YELLOW)
                ok, why = _try_connect(sta, todo[0], todo[1])
                if ok:
                    wifi_ssid, wifi_pass = todo
                    save_creds(todo[0], todo[1])
                    setup_msg("CONNECTED!", todo[0], MINT)
                    time.sleep(2)
                    saved = True
                    break
                setup_msg(why, "try again from phone", ALERT)
                time.sleep(3)
                draw_setup_screen(nets, pw, ip)
    finally:
        try:
            srv.close()
        except Exception:
            pass
        try:
            ap.active(False)
        except Exception:
            pass
    dirty = True
    return saved


# =====================================================================
#  main
# =====================================================================
# =====================================================================
#  ASK CLAUDE: questions on the screen (touch) or typed on a phone
# =====================================================================
ASK_PROVIDER = "gemini"    # "gemini" (free tier) or "claude" (paid API key)
ASK_MODELS = {"gemini": ("gemini-3.1-flash-lite", "gemini-3.5-flash-lite", "gemini-3.5-flash"),
              "claude": ("claude-haiku-5-5",)}
ASK_KEY_FILE = "/claude.json"
ASK_SYSTEM = ("You answer on a tiny round clock screen. Reply in plain ASCII English "
              "only: no markdown, no emoji, no lists, at most 55 words.")
ASK_MIN_GAP_MS = 3000      # minimum time between two questions
ASK_LINES = 10             # text lines per page
ASK_COLS = 22              # characters per line

ASK_PRESETS = (
    ("FUN FACT", "Tell me one surprising fun fact."),
    ("JOKE", "Tell me one short clean joke."),
    ("MOTIVATE", "Give me one short motivational line for today."),
    ("NEW WORD", "Give me one uncommon English word with its meaning and an example."),
    ("OUTFIT", None),
    ("ESP32 TIP", "Give me one practical tip for ESP32 MicroPython projects."),
    ("DINNER", "Suggest one tasty dinner I can make in 15 minutes."),
)

ask_active = False
ask_srv = None
ask_wlan = None
ask_ip = ""
ask_pin = "0000"
ask_mode = 0          # 0 = question menu, 1 = answer pages
ask_sel = 0
ask_page = 0
ask_lines = []
ask_err = False
ask_last = None

_ASCII_MAP = {8216: "'", 8217: "'", 8220: '"', 8221: '"', 8211: "-", 8212: "-", 8230: "..."}


def _ascii(t):
    out = ""
    for ch in t:
        o = ord(ch)
        if o in _ASCII_MAP:
            out += _ASCII_MAP[o]
        elif 32 <= o < 127:
            out += ch
        else:
            out += " "
    return " ".join(out.split())


def _wrap(t, w):
    lines = []
    cur = ""
    for word in t.split(" "):
        while len(word) > w:
            if cur:
                lines.append(cur)
                cur = ""
            lines.append(word[:w])
            word = word[w:]
        if not word:
            continue
        if not cur:
            cur = word
        elif len(cur) + 1 + len(word) <= w:
            cur += " " + word
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def ask_prompt(i):
    p = ASK_PRESETS[i][1]
    if p:
        return p
    if wx_temp is None:
        return "What should I wear on a cool autumn day in Montreal? One sentence."
    return ("It is %d C and %s in %s right now. What should I wear? One sentence."
            % (round(wx_temp), wx_text(wx_code).lower(), WEATHER_CITY.title()))


def ask_key():
    try:
        import json
        with open(ASK_KEY_FILE) as f:
            return json.load(f).get("key", "")
    except Exception:
        return ""


def ask_save_key(k):
    import json
    with open(ASK_KEY_FILE, "w") as f:
        json.dump({"key": k}, f)


def ask_wifi():
    """Makes sure WiFi is on. Returns the IP address, or '' when there is no WiFi."""
    global ask_wlan
    ask_wlan = wifi_connect()
    return ask_wlan.ifconfig()[0] if ask_wlan else ""


def ask_claude(prompt):
    """Asks Claude. Returns (ok, text) - the text is plain ASCII."""
    global ask_ip
    key = ask_key()
    if not key:
        where = ("http://" + ask_ip) if ask_ip else "the clock page"
        return False, "NO API KEY YET. OPEN %s ON YOUR PHONE, ENTER THE PIN AND PASTE YOUR KEY." % where
    if not (ask_wlan and ask_wlan.isconnected()):
        ask_ip = ask_wifi()
        if not ask_ip:
            return False, "NO WIFI CONNECTION."
    import json
    import requests
    gc.collect()
    r = None
    try:
        models = ASK_MODELS[ASK_PROVIDER]
        for mi in range(len(models)):
            model = models[mi]
            if ASK_PROVIDER == "claude":
                url = "https://api.anthropic.com/v1/messages"
                body = json.dumps({"model": model, "max_tokens": 220, "system": ASK_SYSTEM,
                                   "messages": [{"role": "user", "content": prompt}]}).encode()
                hdrs = {"x-api-key": key, "anthropic-version": "2023-06-01",
                        "content-type": "application/json"}
            else:
                url = ("https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent"
                       % model)
                body = json.dumps({"systemInstruction": {"parts": [{"text": ASK_SYSTEM}]},
                                   "contents": [{"role": "user", "parts": [{"text": prompt}]}],
                                   "generationConfig": {"maxOutputTokens": 800,
                                                        "temperature": 0.8}}).encode()
                hdrs = {"x-goog-api-key": key, "content-type": "application/json"}
            r = None
            for attempt in range(2):
                try:
                    r = requests.post(url, data=body, headers=hdrs)
                    break
                except Exception as e:
                    print("ask retry:", e)
                    gc.collect()
                    time.sleep_ms(1200)
            if r is None:
                return False, "NO REPLY FROM THE SERVER. TRY AGAIN."
            if r.status_code == 404 and mi + 1 < len(models):
                r.close()                      # model not available: try the next one
                r = None
                continue
            break
        code = r.status_code
        if code in (401, 403):
            return False, "KEY REJECTED (%d). CHECK YOUR API KEY." % code
        if code == 429:
            return False, "TOO MANY REQUESTS (429). WAIT A MOMENT."
        if code != 200:
            try:
                msg = _ascii(r.json()["error"]["message"])[:90]
            except Exception:
                msg = ""
            return False, ("ERROR %d. %s" % (code, msg)).strip()
        d = r.json()
        if ASK_PROVIDER == "claude":
            txt = "".join(b.get("text", "") for b in d.get("content", [])
                          if b.get("type") == "text")
        else:
            cands = d.get("candidates") or [{}]
            txt = "".join(p.get("text", "") for p in
                          cands[0].get("content", {}).get("parts", []))
        txt = _ascii(txt)[:ASK_LINES * ASK_COLS * 3]
        return (True, txt) if txt else (False, "EMPTY ANSWER.")
    except Exception as e:
        print("ask error:", e)
        return False, "SOMETHING WENT WRONG. TRY AGAIN."
    finally:
        if r is not None:
            try:
                r.close()
            except Exception:
                pass
        gc.collect()


def ask_run(prompt):
    """Shows 'thinking', asks, then switches the screen to the answer pages."""
    global ask_mode, ask_page, ask_lines, ask_err, ask_last, dirty
    fb.fill(BLACK_C)
    big_text_c("THINKING", 96, 2, C_SEC)
    text_c("asking AI...", 124, GRAY)
    lcd.flush(buf)
    ok, txt = ask_claude(prompt)
    ask_err = not ok
    ask_lines = _wrap(txt, ASK_COLS)
    ask_page = 0
    ask_mode = 1
    ask_last = time.ticks_ms()
    dirty = True
    return ok, txt


def _ask_ready(now):
    return ask_last is None or time.ticks_diff(now, ask_last) >= ASK_MIN_GAP_MS


def ask_enter():
    global ask_active, ask_srv, ask_ip, ask_pin, ask_mode, ask_fails
    import os
    ask_fails = 0
    splash("ASK AI", "connecting WiFi...")
    ask_ip = ask_wifi()
    ask_pin = "".join(str(b % 10) for b in os.urandom(4))
    ask_mode = 0
    ask_srv = None
    if ask_ip:
        try:
            import socket
            s = socket.socket()
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", 80))
            s.listen(2)
            s.setblocking(False)
            ask_srv = s
        except Exception as e:
            print("ask server:", e)
    ask_active = True


def ask_leave():
    global ask_active, ask_srv, ask_wlan
    if ask_srv:
        try:
            ask_srv.close()
        except Exception:
            pass
    ask_srv = None
    try:
        import network
        w = network.WLAN(network.STA_IF)
        w.disconnect()
        w.active(False)
    except Exception:
        pass
    ask_wlan = None
    ask_active = False


_ASK_CSS = (_CSS + "textarea{width:100%;font-size:18px;padding:10px;margin:6px 0;"
            "box-sizing:border-box}.a{background:#222;padding:12px;border-radius:6px;"
            "white-space:pre-wrap;margin:10px 0}")


def _ask_page(note="", q=""):
    has = bool(ask_key())
    body = "<h2>Ask the clock AI</h2>" + note
    if has:
        body += ("<form method=post action=/ask><label>PIN (shown on the clock)</label>"
                 "<input name=pin inputmode=numeric autocomplete=off>"
                 "<label>Your question</label>"
                 "<textarea name=q rows=4 maxlength=400>" + _esc(q) + "</textarea>"
                 "<button>Ask</button></form><hr>")
    body += ("<form method=post action=/key><label>PIN (shown on the clock)</label>"
             "<input name=pin inputmode=numeric autocomplete=off>"
             "<label>" + ("Replace API key" if has else "API key (saved only on this clock)") +
             "</label><input name=key type=password autocomplete=off>"
             "<button>Save key</button></form><p><a href=/update>Update clock code</a></p>")
    return ("<!DOCTYPE html><html><head><meta charset=utf-8>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>Ask AI</title><style>" + _ASK_CSS + "</style></head><body>"
            + body + "</body></html>")


def ask_event(kind, zone, now):
    global screen, ask_sel, ask_mode, ask_page
    if kind == "long":
        screen = SCR_CLOCK
        return
    if kind != "tap":
        return
    if ask_mode == 0:
        if zone == 0:
            ask_sel = (ask_sel - 1) % len(ASK_PRESETS)
        elif zone == 2:
            ask_sel = (ask_sel + 1) % len(ASK_PRESETS)
        elif _ask_ready(now):
            ask_run(ask_prompt(ask_sel))
    else:
        pages = (len(ask_lines) + ASK_LINES - 1) // ASK_LINES
        if zone == 1:
            ask_mode = 0
        elif zone == 0:
            if ask_page == 0:
                ask_mode = 0
            else:
                ask_page -= 1
        else:
            if ask_page + 1 >= pages:
                ask_mode = 0
            else:
                ask_page += 1


def draw_ask_screen():
    fb.fill(BLACK_C)
    if ask_mode == 0:
        big_text_c("ASK AI", 22, 2, C_MAIN)
        text_c("%d / %d" % (ask_sel + 1, len(ASK_PRESETS)), 48, GRAY)
        big_text_c(ASK_PRESETS[ask_sel][0], 66, 2, C_SEC)
        big_text("<", 24, 66, 2, C_DATE)
        big_text(">", 200, 66, 2, C_DATE)
        text_c("TAP MIDDLE = ASK", 100, WHITE_C)
        text_c("SIDES = OTHER QUESTION", 112, GRAY)
        if ask_ip:
            text_c("OR TYPE ON YOUR PHONE:", 134, GRAY)
            text_c("http://" + ask_ip, 148, C_DATE)
            text_c("PIN " + ask_pin, 162, YELLOW)
        else:
            text_c("NO WIFI - TOUCH ONLY", 140, YELLOW)
        text_c("HOLD = CLOCK", 196, GRAY)
    else:
        text_c("PROBLEM" if ask_err else "ANSWER", 24, YELLOW if ask_err else C_SEC)
        pages = max(1, (len(ask_lines) + ASK_LINES - 1) // ASK_LINES)
        a = ask_page * ASK_LINES
        for i, ln in enumerate(ask_lines[a:a + ASK_LINES]):
            fb.text(ln, 32, 44 + i * 12, WHITE_C)
        text_c("PAGE %d/%d" % (ask_page + 1, pages), 170, GRAY)
        text_c("SIDES=PAGE MID=MENU", 190, GRAY)


# =====================================================================
#  PACK: alarm, battery page, flip-to-start timer, phone code update
# =====================================================================
ALARM_SNOOZE_MIN = 5
ALARM_RING_MS = 120000
FLIP_UP_SIGN = 1          # set -1 if FLIP TIMER reacts when the screen faces UP
FLIP_G = 0.75             # how flat (in g) the board must be to count as face-down
ALARM_PIN = None          # optional buzzer GPIO (e.g. 17). None = no sound
BATT_PIN = 1              # battery voltage is read on GPIO1
BATT_DIV = 3.0            # voltage divider on the board (measure with a multimeter)
BATT_CAL = 1.0            # fine tune: real voltage / shown voltage
BATT_N = 60               # history points
BATT_HIST_MS = 120000     # one history point every 2 minutes
ASK_MAX_FAILS = 5         # wrong PINs allowed per visit to the ASK page

al_f = 0                  # alarm screen: 0 = on/off, 1 = hour, 2 = minutes
al_ring = False
al_ring_t = 0
al_fired = -1
al_snooze_at = 0
_snd = None
batt_v = 0.0
batt_t = 0
batt_hist_t = 0
batt_n = 0
batt_hist = array("H", [0] * BATT_N)
batt_chg = False
batt_low = False
_low_n = 0
_adc = None
_flip_down = False
_flip_cand = False
_flip_t = 0
ask_fails = 0


# ---- alarm ----------------------------------------------------------
def alarm_check(now, lt):
    global al_ring, al_ring_t, al_fired, al_snooze_at, dirty
    if al_ring:
        if time.ticks_diff(now, al_ring_t) >= ALARM_RING_MS:
            alarm_stop()
        return
    if not time_is_utc:
        return
    due = False
    key = lt[2] * 1440 + lt[3] * 60 + lt[4]
    if al_on and lt[3] == al_h and lt[4] == al_m and key != al_fired:
        al_fired = key
        due = True
    if al_snooze_at and time.time() >= al_snooze_at:
        al_snooze_at = 0
        due = True
    if due:
        al_ring = True
        al_ring_t = now
        dirty = True


def alarm_stop():
    global al_ring, al_snooze_at, dirty
    al_ring = False
    al_snooze_at = 0
    dirty = True
    alarm_sound(False)


def alarm_snooze():
    global al_ring, al_snooze_at, dirty
    al_ring = False
    al_snooze_at = time.time() + ALARM_SNOOZE_MIN * 60
    dirty = True
    alarm_sound(False)


def alarm_sound(on, now=0):
    global _snd
    if ALARM_PIN is None:
        return
    if on:
        if _snd is None:
            _snd = PWM(Pin(ALARM_PIN), freq=2000, duty_u16=0)
        _snd.duty_u16(32768 if (now // 250) % 2 == 0 else 0)
    elif _snd is not None:
        _snd.duty_u16(0)


def alarm_step(zone, now, big=False):
    global al_on, al_h, al_m, last_rep
    d = -1 if zone == 0 else 1
    if al_f == 0:
        al_on = 1 - al_on
    elif al_f == 1:
        al_h = (al_h + d) % 24
    else:
        al_m = (al_m + d * (5 if big else 1)) % 60
    last_rep = now


def alarm_event(kind, zone, now):
    global screen, al_f
    if kind == "long":
        save_settings()
        screen = SCR_CLOCK
    elif kind == "down" and zone != 1:
        alarm_step(zone, now)
    elif kind == "tap" and zone == 1:
        al_f += 1
        if al_f > 2:
            al_f = 0
            save_settings()
            screen = SCR_CLOCK


def draw_alarm_screen():
    fb.fill(BLACK_C)
    col = C_MAIN if al_on else GRAY
    big_text_c("ALARM", 18, 2, col)
    hour, suf, hide0 = fmt_hour(al_h)
    draw_big_time(hour, al_m, 44, col, True, hide0)
    if al_f == 1:
        fb.fill_rect(30, 112, 74, 3, YELLOW)
    elif al_f == 2:
        fb.fill_rect(136, 112, 74, 3, YELLOW)
    if suf:
        text_c(suf.strip(), 119, col)
    big_text_c(ONOFF[al_on], 134, 3, col)
    if al_f == 0:
        big_text("<", 40, 134, 3, YELLOW)
        big_text(">", 176, 134, 3, YELLOW)
    text_c(("SIDES = ON / OFF", "SIDES = HOUR", "SIDES = MINUTES")[al_f], 170, WHITE_C)
    text_c("MIDDLE = NEXT", 184, GRAY)
    text_c("HOLD = SAVE", 198, GRAY)


def draw_ring_screen(now, lt):
    fb.fill(ALERT if (now // 400) % 2 == 0 else BLACK_C)
    big_text_c("ALARM", 22, 2, WHITE_C)
    hour, suf, hide0 = fmt_hour(lt[3])
    draw_big_time(hour, lt[4], 56, WHITE_C, True, hide0)
    if suf:
        text_c(suf.strip(), 124, WHITE_C)
    text_c("TAP = STOP", 150, WHITE_C)
    text_c("SHAKE = SNOOZE %d MIN" % ALARM_SNOOZE_MIN, 168, WHITE_C)


# ---- battery --------------------------------------------------------
_BT = ((3.30, 0), (3.50, 5), (3.60, 12), (3.70, 25), (3.75, 40), (3.80, 50),
       (3.85, 60), (3.90, 70), (3.95, 78), (4.00, 85), (4.10, 93), (4.20, 100))


def batt_pct(v):
    if v <= _BT[0][0]:
        return 0
    for i in range(1, len(_BT)):
        if v <= _BT[i][0]:
            v0, p0 = _BT[i - 1]
            v1, p1 = _BT[i]
            return int(p0 + (p1 - p0) * (v - v0) / (v1 - v0))
    return 100


def batt_read():
    global _adc
    try:
        if _adc is None:
            from machine import ADC
            _adc = ADC(Pin(BATT_PIN), atten=ADC.ATTN_11DB)
        s = 0
        for _ in range(16):
            s += _adc.read_uv()
        return s / 16 / 1000000 * BATT_DIV * BATT_CAL
    except Exception:
        return None


def batt_update(now):
    global batt_v, batt_t, batt_hist_t, batt_n, batt_chg, batt_low, _low_n
    if batt_t and time.ticks_diff(now, batt_t) < (4000 if screen == SCR_BATT else 30000):
        return
    batt_t = now or 1
    v = batt_read()
    if v is None:
        return
    batt_v = v
    if not batt_hist_t or time.ticks_diff(now, batt_hist_t) >= BATT_HIST_MS:
        batt_hist_t = now or 1
        mv = int(v * 1000)
        if batt_n < BATT_N:
            batt_hist[batt_n] = mv
            batt_n += 1
        else:
            for i in range(BATT_N - 1):
                batt_hist[i] = batt_hist[i + 1]
            batt_hist[BATT_N - 1] = mv
        if batt_n >= 3:
            batt_chg = batt_hist[batt_n - 1] - batt_hist[batt_n - 3] >= 15
    if 3.0 < v < 3.5:
        _low_n += 1
    else:
        _low_n = 0
    batt_low = _low_n >= 3 and not batt_chg


def draw_batt_screen():
    fb.fill(BLACK_C)
    big_text_c("BATTERY", 16, 2, C_SEC)
    if batt_v < 2.5:
        big_text_c("NO DATA", 62, 2, YELLOW)
        text_c("NO BATTERY OR NO READING", 90, GRAY)
        text_c("%.2f V" % batt_v, 106, GRAY)
    else:
        p = batt_pct(batt_v)
        col = ALERT if p < 15 else (YELLOW if p < 35 else MINT)
        big_text_c("%d%%" % p, 38, 4, col)
        fb.fill_rect(60, 78, 120, 14, GRAY)
        fb.fill_rect(62, 80, 116, 10, BLACK_C)
        fb.fill_rect(62, 80, 116 * p // 100, 10, col)
        text_c("%.2f V" % batt_v, 98, WHITE_C)
        if batt_chg:
            text_c("CHARGING", 112, MINT)
        elif batt_v > 4.15:
            text_c("FULL OR USB POWER", 112, MINT)
        elif batt_low:
            text_c("LOW - SAVER ON", 112, YELLOW)
    if batt_n >= 2:
        x0, x1, y0, y1 = 40, 200, 130, 158
        px = py = None
        for i in range(batt_n):
            v = batt_hist[i] / 1000
            v = 3.3 if v < 3.3 else (4.25 if v > 4.25 else v)
            x = x0 + (x1 - x0) * i // (batt_n - 1)
            y = y1 - int((v - 3.3) / 0.95 * (y1 - y0))
            if px is not None:
                fb.line(px, py, x, y, C_DATE)
            px, py = x, y
        text_c("LAST %d MIN" % ((batt_n - 1) * (BATT_HIST_MS // 60000)), 164, GRAY)
    else:
        text_c("COLLECTING DATA...", 140, GRAY)
    z = _imu_prev[2] / 8192
    text_c("Z " + ("+" if z >= 0 else "") + "%.2f G" % z, 178, GRAY)
    text_c("SIDES = NEXT PAGE", 196, GRAY)
    text_c("MIDDLE = CLOCK", 208, GRAY)


# ---- flip the board face-down to run the timer ------------------------
def flip_poll(now):
    global _flip_down, _flip_cand, _flip_t
    if not imu_ok:
        return
    z = _imu_prev[2] * FLIP_UP_SIGN
    want = _flip_down
    if z < -FLIP_G * 8192:
        want = True
    elif z > -0.4 * 8192:
        want = False
    if want != _flip_down:
        if want == _flip_cand:
            if time.ticks_diff(now, _flip_t) >= 1000:
                _flip_down = want
                flip_changed(want, now)
        else:
            _flip_cand = want
            _flip_t = now
    else:
        _flip_cand = _flip_down


def flip_changed(down, now):
    global timer_state, dirty
    dirty = True
    if not st_flip:
        return
    if down:
        if timer_state == T_SET and screen == SCR_TIMER:
            timer_start(now)
        elif timer_state == T_PAUSE:
            timer_toggle_pause(now)
    else:
        if timer_state == T_RUN:
            timer_toggle_pause(now)
        elif timer_state == T_DONE:
            timer_state = T_SET


# ---- update the clock program from a phone / PC browser -----------------
OTA_CUR = "/clock_timer.py"
OTA_BAK = "/clock_timer.bak"
OTA_NEW = "/clock_new.py"


def _update_page():
    return ("<!DOCTYPE html><html><head><meta charset=utf-8>"
            "<meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>Update</title><style>" + _CSS + "pre{white-space:pre-wrap}</style></head><body>"
            "<h2>Update the clock code</h2>"
            "<p>Choose the new <b>clock_timer.py</b>, type the PIN shown on the clock and press "
            "Upload. The old program is kept as clock_timer.bak.</p>"
            "<label>PIN (shown on the clock)</label><input id=p inputmode=numeric autocomplete=off>"
            "<input id=f type=file>"
            "<button onclick=up()>Upload and restart</button><pre id=o></pre>"
            "<p><a href=/>Back</a></p>"
            "<script>async function up(){var f=document.getElementById('f').files[0],"
            "o=document.getElementById('o');if(!f){o.textContent='Choose the file first';return}"
            "var b=await f.arrayBuffer(),u=new Uint8Array(b),s=0;"
            "for(var i=0;i<u.length;i++)s+=u[i];o.textContent='Sending '+u.length+' bytes...';"
            "try{var r=await fetch('/ota?pin='+encodeURIComponent(document.getElementById('p').value)"
            "+'&sum='+s,{method:'POST',body:b});o.textContent=await r.text()}"
            "catch(e){o.textContent='Connection lost. If the clock says UPDATED it is restarting.'}}"
            "</script></body></html>")


def _read_head(c):
    c.settimeout(5)
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = c.recv(1024)
        if not chunk:
            break
        data += chunk
        if len(data) > 4096:
            break
    head, _, body = data.partition(b"\r\n\r\n")
    lines = head.split(b"\r\n")
    first = lines[0].decode().split(" ")
    n = 0
    for ln in lines[1:]:
        if ln.lower().startswith(b"content-length:"):
            n = int(ln.split(b":")[1])
    return first[0], (first[1] if len(first) > 1 else "/"), n, body


def _ota_reply(c, msg):
    c.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\n"
              b"Cache-Control: no-store\r\nConnection: close\r\n\r\n" + msg.encode())


def _ota_receive(c, path, n, first):
    global ask_fails, dirty
    import os
    gc.collect()
    q = _parse_form((path.split("?", 1) + [""])[1].encode())
    err = ""
    if ask_fails >= ASK_MAX_FAILS:
        err = "LOCKED: too many wrong PINs. Re-open the ASK page on the clock."
    elif q.get("pin", "") != ask_pin:
        ask_fails += 1
        err = "WRONG PIN (shown on the clock screen)"
    elif not (20000 < n < 160000):
        err = "FILE SIZE LOOKS WRONG (%d bytes)" % n
    else:
        ask_fails = 0
        try:
            sm_want = int(q.get("sum", "-1"))
        except Exception:
            sm_want = -1
        try:
            os.remove(OTA_NEW)
        except OSError:
            pass
        try:
            st = os.statvfs("/")
            if st[0] * st[3] < n + 40000:
                err = "NOT ENOUGH FLASH SPACE"
        except Exception:
            pass
    if err:
        left = n - len(first)
        try:
            while left > 0:
                ch = c.recv(1024)
                if not ch:
                    break
                left -= len(ch)
        except Exception:
            pass
        _ota_reply(c, "ERROR: " + err)
        return
    splash("UPDATE", "receiving code...")
    total = len(first)
    sm = sum(first)
    try:
        with open(OTA_NEW, "wb") as fh:
            fh.write(first)
            while total < n:
                ch = c.recv(min(1024, n - total))
                if not ch:
                    break
                fh.write(ch)
                total += len(ch)
                sm += sum(ch)
    except Exception as e:
        err = "UPLOAD FAILED: %s" % e
    if not err:
        if total != n:
            err = "UPLOAD INCOMPLETE (%d of %d bytes)" % (total, n)
        elif sm != sm_want:
            err = "CHECKSUM MISMATCH - try again"
        else:
            with open(OTA_NEW, "rb") as fh:
                head = fh.readline()
                fh.seek(n - 300)
                tail = fh.read()
            if not head.startswith(b"# Digital Clock") or b"main()" not in tail:
                err = "THIS IS NOT THE CLOCK PROGRAM"
    if err:
        try:
            os.remove(OTA_NEW)
        except OSError:
            pass
        dirty = True
        _ota_reply(c, "ERROR: " + err)
        return
    try:
        os.remove(OTA_BAK)
    except OSError:
        pass
    try:
        os.rename(OTA_CUR, OTA_BAK)
        os.rename(OTA_NEW, OTA_CUR)
    except Exception as e:
        try:
            os.rename(OTA_BAK, OTA_CUR)
        except Exception:
            pass
        dirty = True
        _ota_reply(c, "ERROR: could not install (%s)" % e)
        return
    _ota_reply(c, "OK - %d bytes saved. The clock restarts now." % n)
    splash("UPDATED", "restarting...")
    time.sleep_ms(1000)
    import machine
    machine.reset()


def ask_serve():
    """Handles one waiting phone request (if any)."""
    global ask_fails
    try:
        c, _a = ask_srv.accept()
    except OSError:
        return
    try:
        meth, path, n, body = _read_head(c)
        if meth == "POST" and path.startswith("/ota"):
            _ota_receive(c, path, n, body)
            return
        if meth == "GET" and path.startswith("/update"):
            _send(c, _update_page())
            return
        while len(body) < n and len(body) < 2048:
            chunk = c.recv(1024)
            if not chunk:
                break
            body += chunk
        note = ""
        q = ""
        if meth == "POST":
            f = _parse_form(body)
            q = f.get("q", "").strip()[:400]
            if ask_fails >= ASK_MAX_FAILS:
                note = "<p style=color:#fd5>Too many wrong PINs. Close the ASK page on the clock and open it again.</p>"
            elif f.get("pin", "").strip() != ask_pin:
                ask_fails += 1
                note = "<p style=color:#fd5>Wrong PIN. It is shown on the clock screen.</p>"
            else:
                ask_fails = 0
                if path.startswith("/key"):
                    k = f.get("key", "").strip()
                    if len(k) >= 20 and " " not in k and '"' not in k and "\\" not in k:
                        ask_save_key(k)
                        note = "<p style=color:#7fe0c2>Key saved.</p>"
                    else:
                        note = "<p style=color:#fd5>That does not look like an API key.</p>"
                elif not q:
                    note = "<p style=color:#fd5>Type a question first.</p>"
                elif not _ask_ready(time.ticks_ms()):
                    note = "<p style=color:#fd5>Wait a few seconds and try again.</p>"
                else:
                    ok, txt = ask_run(q)
                    note = "<div class=a>" + _esc(txt) + "</div>"
                    q = ""
        _send(c, _ask_page(note, q))
    except Exception as e:
        print("ask serve:", e)
    finally:
        try:
            c.close()
        except Exception:
            pass


def main():
    global buf, fb, lcd, button, dirty, sync_fails

    button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    global bl
    bl = PWM(Pin(LCD_BL), freq=5000)
    load_settings()
    set_backlight(BRIGHT_LEVELS[st_bright])

    gc.collect()
    buf = bytearray(240 * 240 * 2)     # 115 KB frame buffer
    fb = framebuf.FrameBuffer(buf, 240, 240, framebuf.RGB565)

    setup_touch()
    imu_init()
    lcd = GC9A01()

    splash("CLOCK", "connecting WiFi...")
    load_creds()
    ok = ntp_sync()
    if not ok and wifi_problem and wifi_ssid != "YOUR_WIFI_NAME":
        ok = ntp_sync()                  # one more try (router may still be starting)
    if not ok and wifi_problem:
        if wifi_setup():
            ok = ntp_sync()
    schedule_sync(ok)
    splash("CLOCK", "getting weather...")
    schedule_wx(weather_sync())

    last_sec = -1
    last_draw = 0
    while True:
        now = time.ticks_ms()

        ev = poll_input(now)
        if ev:
            handle_event(ev, now)

        if screen == SCR_ASK and not ask_active:
            ask_enter()
            dirty = True
        elif screen != SCR_ASK and ask_active:
            ask_leave()

        shake, moved = imu_poll(now)
        if shake or moved:
            wake(now)
        if shake:
            shake_event()
        if st_flip:
            flip_poll(now)

        # hold left/right in the timer-set screen = keep stepping
        if screen == SCR_TIMER and timer_state == T_SET and pressed and press_zone != 1:
            if (time.ticks_diff(now, press_t) >= REPEAT_START_MS and
                    time.ticks_diff(now, last_rep) >= REPEAT_EVERY_MS):
                step_minutes(press_zone, now)
                dirty = True

        if screen == SCR_ALARM and al_f != 0 and pressed and press_zone != 1:
            if (time.ticks_diff(now, press_t) >= REPEAT_START_MS and
                    time.ticks_diff(now, last_rep) >= REPEAT_EVERY_MS):
                alarm_step(press_zone, now, time.ticks_diff(now, press_t) >= 2000)
                dirty = True

        update_timer(now)
        batt_update(now)

        if time.ticks_diff(now, next_sync) >= 0 and not pressed and screen != SCR_ASK:
            ok = ntp_sync()
            if ok:
                sync_fails = 0
            elif wifi_problem:
                sync_fails += 1
                if sync_fails >= FAILS_BEFORE_SETUP:
                    sync_fails = 0
                    if wifi_setup():
                        ok = ntp_sync()
            schedule_sync(ok)
            dirty = True

        if (time.ticks_diff(now, next_wx) >= 0 and not pressed and timer_state != T_RUN
                and screen != SCR_ASK):
            schedule_wx(weather_sync())
            dirty = True

        lt = local_now()
        alarm_check(now, lt)
        alarm_sound(al_ring or timer_state == T_DONE, now)
        if al_ring:
            if dirty or time.ticks_diff(now, last_draw) >= 100:
                dirty = False
                last_draw = now
                draw_ring_screen(now, lt)
                lcd.flush(buf)
        elif screen == SCR_CLOCK or screen == SCR_WX or screen == SCR_WORLD or screen == SCR_BATT:
            if lt[5] != last_sec or dirty:
                last_sec = lt[5]
                dirty = False
                if screen == SCR_CLOCK:
                    draw_clock_screen(lt)
                elif screen == SCR_WX:
                    draw_weather_screen()
                elif screen == SCR_BATT:
                    draw_batt_screen()
                else:
                    draw_world_screen(lt)
                lcd.flush(buf)
        elif screen == SCR_ALARM:
            if dirty:
                dirty = False
                draw_alarm_screen()
                lcd.flush(buf)
        elif screen == SCR_SET:
            if dirty:
                dirty = False
                draw_settings_screen()
                lcd.flush(buf)
        elif screen == SCR_ASK:
            if ask_srv:
                ask_serve()
            if dirty:
                dirty = False
                draw_ask_screen()
                lcd.flush(buf)
        else:
            if dirty or time.ticks_diff(now, last_draw) >= 100:
                dirty = False
                last_draw = now
                draw_timer_screen(now)
                lcd.flush(buf)

        set_backlight(want_brightness(now, lt))

        time.sleep_ms(10)


main()
