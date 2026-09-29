# Split-Architecture Pan-Tilt Visual Servoing — Code and Data

This repository holds the code and data for the paper *"A Split-Architecture Mechatronic
Pan-Tilt Tracking System Using Closed-Loop Visual Servoing"*.

The system splits the work between two boards. A Raspberry Pi 5 runs image capture,
detection (HSV colour or Haar cascade) and the soft-zone / step-and-wait control law. An
Arduino Uno R4 WiFi generates PWM for ten SG90 servos: two for the pan/tilt head and eight
for the quadruped legs. It also gates a 5 V laser diode behind a firmware watchdog. The
two boards talk over a 115200-baud USB-CDC link using an ASCII protocol.

## Contents

| Path | What it is |
|---|---|
| `firmware/robot_arduino.ino` | Arduino firmware: turret + leg protocol parser, PWM, laser gate, 500 ms watchdog |
| `host/avci_taret.py` | Raspberry Pi host application: threaded capture, detectors, control law, operator-confirmation FSM, latency logging (`--log`) |
| `data/latency_log.csv` | Raw host log, 13,870 commands (363 s). `lat_ms` = age of the camera frame at the serial write |
| `data/servo_identification/` | Pan-axis identification, 3 runs: bidirectional 0.1° staircase (`*_e1_staircase.csv`), 30 step trials each (`*_e1_step.csv`), fitted parameters (`*_plant_params.json`) |
| `figures/make_figures.py` | Regenerates the latency histogram, the controller ablation and the other figures in the paper (writes to `figures/out/`) |
| `measurement/` | Hardware tracking trials (see below) |

## Running

Host (on the Pi, with the Arduino on `/dev/ttyACM0`):

```bash
pip install opencv-python numpy pyserial
python3 host/avci_taret.py --log        # writes latency_log.csv
```

Figures:

```bash
pip install numpy matplotlib
python figures/make_figures.py
```

## Tracking trials (`measurement/`)

- `hedef_ekrani.py` runs on any computer placed in front of the robot. It shows a red
  disc whose motion is set in degrees as seen from the camera. Modes: `sabit` (static),
  `sinus` (sine), `tarama` (sweep with rising speed), `sicrama` (jumps plus a corner
  flash, for high-speed-video latency measurement). Each run's motion is logged to CSV.
- `takip_denemesi.py` runs on the Pi. It runs one control law (`SOFT` = the paper's
  law, `PID`, `FLAT`) against the target, logs every processed frame, and saves every
  Nth frame for ground-truth annotation.
- `isaretle.py` is the click-to-annotate tool that marks the true target centre on the
  saved frames.
- `rig.py` and `controllers.py` provide the shared camera, detector, serial and control
  code.

Examples:

```bash
python hedef_ekrani.py --mod sinus --genlik 8 --frekans 0.2 --mesafe-cm 100 --ekran-cm 34.5
python3 takip_denemesi.py --law SOFT --cond S2 --freq 0.2 --run 1 --port /dev/ttyACM0
python isaretle.py --kok veri_takip --adet 200
```

Some identifiers and comments are in Turkish (for example *sabit* = static,
*tarama* = sweep, *takip denemesi* = tracking trial).

## License

MIT (see `LICENSE`). The Haar cascade file `haarcascade_frontalface_default.xml`
ships with OpenCV and is not included here.
