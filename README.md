# ESP32-S3 Bunny Pomodoro Clock

A MicroPython clock / Pomodoro timer for the **Waveshare ESP32-S3-Touch-LCD-1.28**
(GC9A01 round LCD, CST816S touch, QMI8658 IMU, battery header) plus a
**3D-printable bunny case** for it.

```
firmware/clock_timer.py      the whole firmware (single file)
case/stl/                    printable parts (STL)
case/source/                 OpenSCAD source of the case
```

## Firmware features
- Digital clock (3 styles + analog), NTP time, North America daylight saving, 12/24 h
- Timer (1-99 min); optional **flip face-down to start**, lift to pause (Pomodoro style)
- **Alarm**: flashing screen, tap = stop, shake = snooze 5 min (optional buzzer on `ALARM_PIN`)
- Weather page (Open-Meteo, no key) and a second world clock (Baghdad)
- **Battery page**: voltage, estimated %, 2-hour graph, automatic low-battery dimming
- **ASK AI** page: preset questions on the screen or free text from a phone (Gemini free tier or Claude)
- WiFi setup portal, themes, brightness, night dim, shake/motion wake, auto-rotate
- **OTA**: upload a new `clock_timer.py` from a browser at `http://<clock-ip>/update` (PIN protected)

### Install
1. Flash MicroPython (ESP32_GENERIC_S3, v1.27 tested) to the board.
2. Copy `firmware/clock_timer.py` to the board root (Thonny: *Save as -> MicroPython device*).
3. Start it from `boot.py` / `main.py` with `import clock_timer`.
4. First start: join the **Clock-Setup** WiFi from your phone and enter your WiFi details
   (saved on the board in `/wifi.json`).

### Touch guide
Clock: tap middle = timer, right = weather -> Baghdad -> battery -> ASK AI, left = alarm,
hold = settings. Hold = back to the clock on most pages.

### ASK AI
Open the ASK page, browse to the address shown on the clock, enter the PIN shown on the screen
and paste your own API key (Google AI Studio). The key is stored only on the board
(`/claude.json`) - never commit that file. MicroPython `requests` does not verify TLS
certificates, so use a restricted key.

### Notes
- Battery divider is `BATT_DIV = 3.0` (GPIO1); calibrate with `BATT_CAL` using a multimeter.
- `FLIP_UP_SIGN = -1` reverses the face-down detection if it reacts the wrong way.
- Tested on MicroPython v1.27.0.

## Bunny case
Two-piece printable bunny enclosure for the board: a front shell with the screen opening and a
rear cover (USB opening, screw posts), plus a stand.

| File | What |
|---|---|
| `case/stl/bunny-front-v9.stl` | front shell (latest iteration) |
| `case/stl/bunny-rear-v9.stl` | rear cover (latest iteration) |
| `case/stl/bunny-stand-v1.stl` | stand |
| `case/stl/bunny-board-fit-test.stl` | quick test print for the screen pocket / USB alignment |
| `case/source/bunny_pomodoro_case_v6.scad` | OpenSCAD source (an earlier iteration than the v9 STLs) |

Tips (from the earlier design notes):
- Print the fit test first. The glass should enter without force and not rattle. If too tight,
  increase `glass_clearance` in the SCAD file by 0.15-0.25 mm.
- 2 x M2 self-tapping screws (about 22-25 mm). Keep screws and bare metal away from the PCB.
- Suggested print settings: PLA, 0.16-0.20 mm layers, 3 walls, 15% gyroid infill; front shell with the
  visible face on the plate, rear cover with its flat back on the plate.
- Board revisions can differ in component height - check rear clearance before tightening.
