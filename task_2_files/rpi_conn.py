"""Task 2 (30 s) - RPi side of the hardcoded run.

The STM does EVERYTHING (moves, IR/ultrasonic decisions, timing). This script only:
  1. loads the camera + YOLO model,
  2. (optionally) sends BI / PW<pwm> / PA<pwm> to the STM,
  3. sends GO,
  4. answers the STM's "T2 SNAP <n>" with "AL<conf%>" or "AR<conf%>",
  5. prints every line the STM sends (moves, IR / US readings, result) with a timestamp.

Default output: ONLY the two ultrasonic readings (obstacles 1 and 2), obstacle 2's length
(time x speed + speed add + offset) and the trig plan, each printed the moment the STM sends it.
--verbose prints every STM line as before.

Usage (on the RPi, in this folder):
    python3 rpi_conn.py                  # BI, then press Enter to GO
    python3 rpi_conn.py --now            # GO straight away (no Enter)
    python3 rpi_conn.py --pwm 3400        # fixed-distance FW PWM (default DRIVE_PWM = 3400)
    python3 rpi_conn.py --ir-pwm 3000     # PWM of every "drive until IR" leg (default IR_PWM = 3000)
    python3 rpi_conn.py --offset 8       # obstacle 2 length offset, cm (default LEN_OFFSET_CM)
    python3 rpi_conn.py --speed-add 10   # added to time x speed, cm (default LEN_SPEED_ADD_CM)
    python3 rpi_conn.py --ac-pwm 3400     # AC approach PWM (default AC_PWM = 3400)
    python3 rpi_conn.py --no-bi          # skip the gyro bias at the start
    python3 rpi_conn.py --verbose        # also show the STM's BRAKE lines
PW / PI / PA are sent EVERY run from DRIVE_PWM / IR_PWM / AC_PWM below, so the STM's own
defaults (T2_DRIVE_PWM 3500, T2_IR_PWM 2800, T2_AC_PWM 3500) are not used.
Ctrl+C sends STOP to the STM.

Photos (with boxes drawn) are saved as photos/t2_30s_<run>_snap<n>_<label>_<conf>.jpg
"""
import argparse
import glob
import os
import queue
import signal
import stat
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

# =============================================================================
# EDIT HERE
# =============================================================================
PORT = "/dev/ttyACM0"          # else the first /dev/ttyACM* or /dev/ttyUSB* is used
BAUD = 115200
MODEL_NAME = "best_ncnn_model"  # looked for here, one folder up, and in the home folder
FRAME_SIZE = (1120, 840)
MODEL_MIN_CONF = 0.10           # YOLO drops boxes below this
DEFAULT_ARROW = "LEFT"          # sent when no arrow is seen at all ("LEFT" or "RIGHT")
FLUSH_FRAMES = 1                # frames thrown away before the real snap (stale buffer)
RUN_TIMEOUT_S = 120             # give up waiting for "T2 END" after this long
PHOTO_DIR = "photos"
ARROW_CLASSES = {"Left Arrow": "LEFT", "Right Arrow": "RIGHT"}
DRIVE_PWM = 3400                # every fixed FW of the run, sent as PW<pwm>   (2399..4199)
AC_PWM = 3400                   # AC40 / AC30 / AC25 approach, sent as PA<pwm> (final AC20 is fixed 3400 in main.c)
IR_PWM = 3000                   # "drive until IR" legs (1st OLL, HOME, PASS1), sent as PI<pwm>
                                #   the move after the 180 (2nd OLL) is fixed 3000 in main.c (T2_ALONG2_PWM)
LEN_OFFSET_CM = 8               # added to obstacle 2 length when START:BESIDE, sent as PO<cm>
LEN_SPEED_ADD_CM = 0            # added to time x speed, sent as PE<cm> (0: TxV was accurate at PI3000)
# =============================================================================


def find_model():
    for d in (HERE, os.path.dirname(HERE), os.path.expanduser("~")):
        path = os.path.join(d, MODEL_NAME)
        if os.path.exists(path):
            return path
    raise FileNotFoundError(f"{MODEL_NAME} not found in {HERE}, its parent or the home folder")


def find_port(wanted):
    if os.path.exists(wanted):
        return wanted
    others = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return others[0] if others else wanted


def free_camera(timeout_s=4.0):
    """Stop any leftover program still holding the camera (e.g. an old run)."""
    devs = set()
    for p in glob.glob("/dev/media*") + glob.glob("/dev/video*"):
        try:
            devs.add(os.stat(p).st_rdev)
        except OSError:
            pass
    me, holders = os.getpid(), []
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            for fd in os.listdir(f"/proc/{d}/fd"):
                try:
                    st = os.stat(f"/proc/{d}/fd/{fd}")
                except OSError:
                    continue
                if stat.S_ISCHR(st.st_mode) and st.st_rdev in devs:
                    holders.append(int(d))
                    break
        except OSError:
            continue
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        if not holders:
            return
        for pid in holders:
            pass  # quiet: stopping a leftover camera user
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        end = time.time() + timeout_s
        while time.time() < end:
            holders = [p for p in holders if os.path.exists(f"/proc/{p}")]
            if not holders:
                time.sleep(0.5)
                return
            time.sleep(0.1)


class Camera:
    def __init__(self, run):
        from picamera2 import Picamera2
        from ultralytics import YOLO
        import cv2
        self.cv2, self.run = cv2, run
        free_camera()
        self.cam = Picamera2()
        self.cam.preview_configuration.main.size = FRAME_SIZE
        self.cam.preview_configuration.main.format = "BGR888"
        self.cam.preview_configuration.align()
        self.cam.configure("preview")
        self.cam.start()
        time.sleep(1.0)
        self.model = YOLO(find_model(), task="detect")
        self.model(self.cam.capture_array(), conf=MODEL_MIN_CONF, imgsz=416, verbose=False)  # warm-up

    def snap(self, n):
        """Returns (label or None, conf, photo path). Never raises."""
        try:
            for _ in range(FLUSH_FRAMES):
                self.cam.capture_array()
            frame = self.cam.capture_array()
            res = self.model(frame, conf=MODEL_MIN_CONF, iou=0.45, imgsz=416, verbose=False)[0]
            names, best, img = self.model.names, None, frame.copy()
            for i in range(len(res.boxes or [])):
                name = names[int(res.boxes.cls[i])]
                conf = float(res.boxes.conf[i])
                x1, y1, x2, y2 = (int(v) for v in res.boxes.xyxy[i].tolist())
                arrow = ARROW_CLASSES.get(name)
                colour = (0, 255, 0) if arrow else (160, 160, 160)
                self.cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
                self.cv2.putText(img, f"{name} {conf:.2f}", (x1, max(12, y1 - 5)),
                                 self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
                if arrow and (best is None or conf > best[1]):
                    best = (arrow, conf)
            label, conf = best if best else (None, 0.0)
            tag = f"{label}_{conf:.2f}" if label else "none"
            path = os.path.join(PHOTO_DIR, f"t2_30s_{self.run}_snap{n}_{tag}.jpg")
            self.cv2.imwrite(path, img)
            return label, conf, path
        except Exception as e:
            return None, 0.0, f"(camera error: {e})"

    def close(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass


class Stm:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.q = queue.Queue()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(256)
            except Exception as e:
                self.q.put(f"__ERR__ {e}")
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self.q.put(line)

    def write(self, text):
        self.ser.write((text + "\n").encode())
        self.ser.flush()

    def ask(self, cmd, starts, timeout=10.0, verbose=False):
        """Send cmd, return the first reply starting with one of `starts`. Quiet unless it fails."""
        self.write(cmd)
        if verbose:
            print(f">> {cmd}")
        end = time.time() + timeout
        while time.time() < end:
            try:
                line = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            if verbose:
                print(f"   {line}")
            if line.startswith(starts):
                if line.startswith("ERR") and not verbose:
                    print(f"!! {cmd}: {line}")
                return line
        print(f"!! no reply to {cmd}")
        return None


def kv(line):
    """'A:1 B:x' -> {'A': '1', 'B': 'x'}"""
    out = {}
    for tok in line.split():
        if ":" in tok:
            k, v = tok.split(":", 1)
            out[k] = v
    return out


def cm(mm_text):
    return int(mm_text) / 10.0


def show(line, state):
    """Print only: ultrasonic of obstacles 1 & 2, obstacle 2 length, trig plan, failures."""
    p = line.split()
    if len(p) >= 3 and p[0] == "T2" and p[2].startswith("AC") and "US:" in line:
        state["ac"] += 1
        if state["ac"] <= 2:
            us = int(kv(line)["US"].replace("mm", ""))
            val = f"{us / 10:.1f} cm" if us >= 0 else "no echo"
            print(f"Ultrasonic obstacle {state['ac']} ({p[2]}): {val}", flush=True)
    elif len(p) >= 3 and p[0] == "T2" and p[2] == "OBS2":
        d = kv(line)
        if "LEN_MM" in d:
            note = {"TxV_FALLBACK": "  !! encoder length unusable - used time x speed",
                    "ENC_FALLBACK": "  !! time x speed unusable - used encoder",
                    "PARTIAL": "  !! end of obstacle 2 not passed - length so far",
                    "DEFAULT": "  !! obstacle 2 not measured - used DEFAULT length (T2_OBS2_DEFAULT_CM)",
                    }.get(d.get("SRC", ""), "")
            print(f"Obstacle 2 length: {cm(d['LEN_MM']):.1f} cm  = {d['SRC']} {cm(d['RAW_MM']):.1f}"
                  f" + speed add {cm(d['ADD_MM']):.1f} + offset {cm(d['OFS_MM']):.1f}"
                  f"  (start {d.get('START', '?')}){note}", flush=True)
        else:
            print("Obstacle 2 length: NOT MEASURED (edge not found) - no trig correction", flush=True)
    elif len(p) >= 3 and p[0] == "T2" and p[2] == "PLAN":
        d = kv(line)
        cmd = d.get("CMD", "NONE").replace("_", " ")
        if "GAP_NOW_MM" not in d:
            print(f"Trig plan: {cmd} ({d.get('REASON', '')})", flush=True)
            return
        fit = d["FIT"]
        fwd, bud = cm(d["FWD_MM"]), cm(d["BUDGET_MM"])
        if fit == "OK":
            note = f"within {fwd:.1f} cm forward (budget {bud:.0f})"
        elif fit == "OVER":
            note = f"within {fwd:.1f} cm forward - OVER budget {bud:.0f} by {fwd - bud:.1f} cm"
        else:
            note = f"within {fwd:.1f} cm forward - 10-20 cm NOT reachable, nearest possible"
        print(f"Trig plan: {cmd} | IR to obstacle 1 edge: now {cm(d['GAP_NOW_MM']):.1f} cm"
              f" -> expected {cm(d['GAP_AFTER_MM']):.1f} cm | correcting {note}", flush=True)
    elif line.startswith("T2 END FAIL") or line.startswith("DONE STOP"):
        print(line, flush=True)


def next_run():
    nums = [int(n.split("_")[2]) for n in os.listdir(PHOTO_DIR)
            if n.startswith("t2_30s_") and n.split("_")[2].isdigit()]
    return max(nums, default=0) + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pwm", type=int, default=DRIVE_PWM, help="fixed-distance FW PWM (2399..4199), sent as PW<pwm>")
    ap.add_argument("--ir-pwm", type=int, default=IR_PWM, help="drive-until-IR PWM (2399..4199), sent as PI<pwm>")
    ap.add_argument("--offset", type=int, default=LEN_OFFSET_CM, help="obstacle 2 length offset cm, sent as PO<cm>")
    ap.add_argument("--speed-add", type=int, default=LEN_SPEED_ADD_CM, help="time x speed add cm, sent as PE<cm>")
    ap.add_argument("--ac-pwm", type=int, default=AC_PWM, help="AC approach PWM (2399..4199), sent as PA<pwm>")
    ap.add_argument("--now", action="store_true", help="GO without waiting for Enter")
    ap.add_argument("--no-bi", action="store_true", help="skip the gyro bias (BI) before GO")
    ap.add_argument("--verbose", action="store_true", help="print every STM line (default: only the summary lines)")
    ap.add_argument("--port", default=PORT)
    a = ap.parse_args()

    os.makedirs(PHOTO_DIR, exist_ok=True)
    run = next_run()
    cam = Camera(run)
    stm = Stm(find_port(a.port))
    try:
        v = a.verbose
        if not a.no_bi:
            stm.ask("BI", ("BIAS", "ERR"), verbose=v)
        stm.ask(f"PW{a.pwm}", ("T2PWM", "ERR"), verbose=v)
        stm.ask(f"PI{a.ir_pwm}", ("T2PWM", "ERR"), verbose=v)
        stm.ask(f"PA{a.ac_pwm}", ("T2PWM", "ERR"), verbose=v)
        stm.ask(f"PO{a.offset}", ("T2PWM", "ERR"), verbose=v)
        stm.ask(f"PE{a.speed_add}", ("T2PWM", "ERR"), verbose=v)
        if not a.now:
            input("Ready. Press Enter to GO ...")

        stm.write("GO")
        t0 = time.time()
        state = {"ac": 0}
        if a.verbose:
            print(f"{0.0:6.2f}  >> GO")
        while time.time() - t0 < RUN_TIMEOUT_S:
            try:
                line = stm.q.get(timeout=0.1)
            except queue.Empty:
                continue
            t = time.time() - t0
            if line.startswith("__ERR__"):
                print(f"{t:6.2f}  !! serial error: {line[8:]}")
                break
            if a.verbose:
                print(f"{t:6.2f}  {line}", flush=True)
            else:
                show(line, state)
            if line.startswith("IMU ") or line.startswith("BIAS"):
                # STM start-up lines in the middle of a run = the STM rebooted (power dip / reset)
                print(f"!! STM REBOOTED mid-run at {t:.1f} s ({line}) - run lost, check battery / wiring",
                      flush=True)
                break

            if line.startswith("T2 SNAP"):
                n = line.split()[-1]
                ts = time.time()
                label, conf, path = cam.snap(n)
                ms = (time.time() - ts) * 1000
                if label is None and a.verbose:
                    print(f"{time.time() - t0:6.2f}  [SNAP {n}] no arrow seen -> DEFAULT {DEFAULT_ARROW}"
                          f"  ({ms:.0f} ms)  {path}")
                if label is None:
                    label, conf = DEFAULT_ARROW, 0.0
                elif a.verbose:
                    print(f"{time.time() - t0:6.2f}  [SNAP {n}] {label}  conf {conf:.2f}"
                          f"  ({ms:.0f} ms)  {path}")
                stm.write(f"A{label[0]}{int(round(conf * 100))}")
            elif line.startswith("T2 END") or line.startswith("DONE STOP"):
                break
        else:
            print(f"!! no 'T2 END' after {RUN_TIMEOUT_S} s - sending STOP")
            stm.write("STOP")
        if a.verbose:
            print(f"Total on the Pi clock: {time.time() - t0:.2f} s")
    except KeyboardInterrupt:
        print("\nCtrl+C - STOP sent to the STM")
        stm.write("STOP")
        time.sleep(0.3)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
