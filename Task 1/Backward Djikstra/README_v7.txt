Laptop :  python algo_server.py
Pi     :  python3 rpi_client.py <laptop ip>
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

