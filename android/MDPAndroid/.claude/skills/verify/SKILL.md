---
name: verify
description: Build, install, and drive the MDPAndroid app on the local emulator to visually verify a change.
---

# Verifying MDPAndroid on-device

The team's physical target device is a **Samsung Galaxy Tab A7 Lite** (1340x800,
~216ppi, 8.7"). Its exact Android/API version is unconfirmed — check Settings →
About tablet → Software information on the real unit when available and update
this note. Two AVDs exist locally, both using the same already-downloaded API 30
(`android-30/google_apis_playstore/arm64-v8a`) system image — no network needed:

- **`TabA7Lite_sim`** — custom `hw.lcd.width=1340 hw.lcd.height=800 hw.lcd.density=216`,
  matching the real device's screen geometry. **Prefer this one** for anything
  layout-related; it's what caught that `Small_Tablet` was misleadingly oversized.
- **`Small_Tablet`** (1920x1200, 320dpi) — a generic wide/landscape tablet profile.
  Good for stress-testing "does this break on an unusually large screen," but not
  representative of the real hardware.

Both are API 30 (Android 11), so neither exercises the `BLUETOOTH_SCAN`/
`BLUETOOTH_CONNECT` runtime-permission path used on Android 12+ — only the older
`ACCESS_FINE_LOCATION` path. If the real tablet turns out to run Android 12+, that
path is only statically reviewed, not runtime-verified, until a matching system
image is available (fetching one from within this sandbox was too slow over the
network last tried — do it from a normal Terminal outside the sandbox instead, or
directly on the real device).

## Boot an emulator

```bash
export ANDROID_HOME=~/Library/Android/sdk
export PATH=$ANDROID_HOME/platform-tools:$ANDROID_HOME/emulator:$PATH
nohup emulator -avd TabA7Lite_sim -no-window -no-audio -no-boot-anim \
  -gpu swiftshader_indirect -port 5556 > emulator.log 2>&1 &
disown
adb -s emulator-5556 wait-for-device
until [ "$(adb -s emulator-5556 shell getprop sys.boot_completed 2>/dev/null | tr -d '\r')" = "1" ]; do sleep 3; done
```

Cold boot takes roughly a minute. `adb devices` should then show
`emulator-5556	device` (not `offline`). Both AVDs can run at once on different
`-port`s (`Small_Tablet` defaults to 5554) if you want to compare side by side —
just pass `-s emulator-XXXX` to every `adb` call below.

## Build, install, launch

```bash
cd /Users/dave/MDP/android/MDPAndroid
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb shell am force-stop com.example.mdpandroid   # clears in-memory app state between runs
adb shell am start -n com.example.mdpandroid/.MainActivity
```

## Driving it

- `TabA7Lite_sim` screen is 1340x800: tabs at roughly Control (220,150), Arena
  (670,150), Manual (1115,150). `Small_Tablet` is 1920x1200 with tabs around
  Control (320,223), Arena (960,223), Manual (1600,223) — always confirm with
  `adb -s <device> shell wm size` rather than assuming.
- Demo mode chip on Control tab (~156,431) enables the movement controls / Send
  buttons without a real Bluetooth connection — commands still route through
  `BluetoothController.send()` and get logged as "Not connected; command not sent: X",
  which is expected and lets you confirm exactly what would have been transmitted.
- **The arena canvas consumes its own drag gestures.** A `swipe` that starts inside
  the canvas bounds gets eaten by its obstacle-drag handler, not treated as a page
  scroll. To scroll the Arena tab, swipe from outside the canvas — the left label
  gutter (`x≈40`) or, once the canvas is off-screen, anywhere.
- Screenshot: `adb shell screencap -p /sdcard/shot.png && adb pull /sdcard/shot.png <local>`, then Read the local file.
- Text fields: `input tap` to focus, then `input text "..."`. Avoid
  `input keyevent --longpress KEYCODE_MOVE_END` to select-to-end — on this AVD it
  triggered a system shortcut into Settings instead of moving the cursor. Clearing a
  field reliably: tap it, `KEYCODE_DEL` a few times, then `input text`.
- Crash check: `adb logcat -d "*:E" | grep -i "mdpandroid\|AndroidRuntime\|FATAL"`
  (quote `*:E` — an unquoted glob gets expanded by the shell and errors).

## Known limitation

**Bluetooth cannot be verified this way at all.** The emulator has no Bluetooth
radio, so C.1/C.2/C.8 (actual pairing/connect/reconnect with the AMD Tool or a
robot) can only be verified on real hardware. Everything else — UI layout, arena
grid rendering, obstacle placement, Demo-mode robot movement, the outgoing command
strings logged via "Not connected; command not sent: ..." — is fully verifiable
here.
