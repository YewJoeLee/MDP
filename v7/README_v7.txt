v7 - CASE C backup photos (FAR, then one angled photo)
=======================================================

Copy this whole folder to the laptop (algo_server.py side) AND to the Pi.
Both sides read the same config.py.

Laptop :  python algo_server.py
Pi     :  python3 rpi_client_android.py <laptop ip>
Camera :  python3 test_camera.py --loop
Replay :  python3 rpi_stm_conn.py -f cmds/latest.txt   (uses cmds/latest.txt.targets.json)

"count" = boxes whose class has a numeric image ID. "Bounding box" is never
counted (it stays in the mapping, it is just not an image ID).

  Snap 1 IDEAL at the planned photo pose (30 cm preferred)
  CASE A  count 1, conf >= 0.50 -> use it                                   (1 photo)
  CASE B  count 1, conf <  0.50 -> BW to FAR, Snap 2, back.
                                   Rank every box of IDEAL + FAR, top one.  (2 photos)
  CASE C  count 0               -> BACKUP 1: BW to FAR (max bound that keeps
                                   clearance), Snap 2. Any box >= 0.50 -> use it.
                                   BACKUP 2: where is the obstacle's dark body?
                                   (FAR photo first, then IDEAL)
                                     RIGHT  -> RT20 (camera swings right)
                                     LEFT   -> LT20 (camera swings left)
                                     CENTRE -> stop (aimed right, turning will not help)
                                     none   -> stop ("no left/right side found, cannot scan")
                                   From the planned turn spot: turn, Snap 3, undo
                                   with XR20 / XL20, drive back to IDEAL.
                                   No spot clears -> stop ("no angled pose found").
                                   Box >= 0.50 -> use it, else nothing.     (max 3 photos)
                                   Weak boxes are printed and logged, never sent.
  CASE D  count 2+              -> FW to NEAR, Snap 2, back.
                                   Rank every box of IDEAL + NEAR, top one. (2 photos)

  Android only gets a TARGET when conf >= ANDROID_MIN_CONF (0.50).
  The robot always ends back on the IDEAL pose, so the plan continues as normal.

Photos : photos/Trial_<x>_Obstacle_<y>_Snap<1-3>_<IDEAL|FAR|NEAR|ANGLED-RT20|ANGLED-LT20>.jpg
Log    : photos/Trial_<x>_diagnosis.txt  - per obstacle: WHY (the step-by-step
         reason it sent / did not send), every photo, every box, obstacle body side.
Terminal: per obstacle ">>> Snap 1 / BACKUP 1 / BACKUP 2" steps, then a WHY list
          and "Android : SENT ..." or "no TARGET sent".

Angled pose (planner.angled_options, sent in each segment target["angled"]):
  start spot = the FAR spot first, then 1 cm closer each try, down to IDEAL.
  The turn arc, the undo arc and the straight drive back to IDEAL must all keep
  MIN_CLEARANCE_CM and stay in the arena. The first spot that works is used.

config.py (new in v7):
  ANDROID_MIN_CONF   = 0.50
  ANGLE_TURN_DEG     = 20      STM needs >= ~20 (smaller turns end at angle - 14..18 = nothing)
  BODY_CENTRE_BAND   = 0.15    body within +-15 % of frame width of the middle = centred
  BODY_MIN_AREA_PX   = 800
  SMALL_TURN_MODELS  RT20 / LT20 / XR20 / XL20 as (forward cm, right cm, clockwise deg).
                     PLACEHOLDERS = the 90-degree models cut at 20 degrees.
                     Measure the real turns (10 runs each) and replace them.
                     With the placeholders, RT20 then XR20 is predicted to end
                     ~6-7 cm further back than it started; the Pi drives that
                     extra distance back to IDEAL.

FAR / NEAR bounds and their "wanted / blocked by / using" explanation are the same as v6.
