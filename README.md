# MDP Robot — Run Guide

Two machines. The **laptop** plans the route, the **Pi** drives the robot.

```
        LAPTOP (server)                         PI (client)
   +----------------------+              +----------------------+
   |  algo_server.py      |  TCP 5000    |  fake_rpi_client.py  |
   |    uses planner.py   | <==========> |    the main script   |
   +----------------------+              +----------+-----------+
                                                    |
        obstacles  -------->                        | imports / calls
                   <--------  commands              |
                                       +------------+------------+
                                       |            |            |
                                 test_maps.py  rpi_stm_conn.py  camera
                                  (data only)         |        + Android
                                                      v
                                                STM32 (UART)
```

Everything on the Pi is driven by `fake_rpi_client.py`. `test_maps.py` is a plain data
file it imports, `rpi_stm_conn.py` is the serial layer it calls, and the camera and
Android messaging live inside the client itself.

### Which files do I actually need to download?

**Only the two laptop files: `algo_server.py` and `planner.py`.**

The three Pi files — `fake_rpi_client.py`, `rpi_stm_conn.py` and `test_maps.py` —
are already on the Pi in the `mdp` folder. If you're setting up, you don't need to
download or copy those. You only touch them if someone has *edited* one and the
change needs to go across, which for a normal setup won't be the case.

If you do need to push an edited file to the Pi, from PowerShell:

```powershell
pscp C:\Users\YourName\Downloads\fake_rpi_client.py pi@192.168.x.x:/home/pi/mdp/
```

---

## 1. Laptop — server side

### Files needed

Only two. Put them in the same folder anywhere on the laptop.

| File | What it does |
|---|---|
| `algo_server.py` | Listens on TCP port 5000. Takes the obstacle list from the Pi, calls the planner, sends back the command list. |
| `planner.py` | The actual path planning. Works out the visit order and turns it into `FW50` / `RT90` / `SNAP6` style commands. |

Nothing else is required — no camera libraries, no pyserial. Plain Python 3 is enough.

### Start the server (PowerShell)

```powershell
cd C:\path\to\your\folder
python algo_server.py
```

You should see:

```
Algo server listening on port 5000...
This laptop appears to be 192.168.x.x
On the Pi run:  python3 fake_rpi_client.py 192.168.x.x
```

**Write down that IP address** — the Pi needs it.

Leave this window open for the whole session. It logs every request and prints the visit order and command count for each run.

If the address it prints looks wrong (e.g. the Pi is on Wi-Fi but this shows an Ethernet address), check with:

```powershell
ipconfig
```

and use the IPv4 address of the **wireless** adapter.

### Firewall

Windows blocks inbound TCP 5000 by default, which makes the Pi time out. Open an **Administrator** PowerShell (right-click → Run as administrator) and run:

```powershell
New-NetFirewallRule -DisplayName "MDP Algo Server" -Direction Inbound -Protocol TCP -LocalPort 5000 -Action Allow
```

This is a one-time setup — the rule stays after a reboot, so you don't need to repeat it every session.

Check it's there:

```powershell
Get-NetFirewallRule -DisplayName "MDP Algo Server"
```

Remove it when the project is done:

```powershell
Remove-NetFirewallRule -DisplayName "MDP Algo Server"
```

---

## 2. Pi — client side

> **Everything in this section is typed in PuTTY, not PowerShell.**
> Connect to the Pi first — see below — then run the commands.

### Connect to the Pi

**Step 1 — same Wi-Fi network.** The laptop and the Pi must be on the *same*
network or they can't see each other. If the Pi runs its own hotspot, join that
from the laptop. If both join a router, join the same one.

**Step 2 — log in over SSH.** Open PuTTY on the laptop:

- Host Name: the Pi's IP address (e.g. `192.168.x.x`)
- Port: `22`
- Connection type: SSH
- Click **Open**

Enter the Pi's username and password when prompted. You'll land in the home
directory, and the prompt will look something like:

```
pi@raspberrypi:~ $
```

From here on, every command below goes in this PuTTY window.

If PuTTY can't connect, the two machines aren't on the same network — check that
before anything else.

### Files on the Pi

Everything is already there, inside the `mdp` folder. Nothing to install or copy.

| File | What it does |
|---|---|
| `fake_rpi_client.py` | The main script. Sends the obstacle list to the laptop, gets the command list back, writes it to `cmds/`, then runs it: movement commands go to the STM, `SNAP` commands trigger the camera. Also runs YOLO on each capture and sends `TARGET,<obstacle>,<image_id>` to the Android tablet over `/dev/rfcomm0`. |
| `rpi_stm_conn.py` | The serial layer. Opens `/dev/ttyACM0` at 115200, sends one command at a time and waits for the STM to reply `DONE` before sending the next. Imported by the client — you don't normally run it yourself. |
| `test_maps.py` | The obstacle list. Format: `(image_id, x, y, side)` where side is `N`/`S`/`E`/`W` — the face the image is on. This is the file you edit between runs. |

### Run it

In the PuTTY session you opened above:

```bash
cd mdp
python3 fake_rpi_client.py 192.168.x.x
```

Use the IP the server printed. That's the whole run — plan, movement, camera, Android, all in one command.

### Changing the obstacle layout

```bash
cd mdp
nano test_maps.py
```

Edit the coordinates, then **Ctrl+O**, **Enter** to save and **Ctrl+X** to exit. No restart needed on the laptop side — the server reads whatever the Pi sends each time.

---

## 3. Order of operations

1. Laptop + Pi: on the **same Wi-Fi network**
2. Laptop: firewall rule added (one-time, see section 1)
3. Laptop, PowerShell: `python algo_server.py`, note the IP it prints
4. Laptop: open **PuTTY**, SSH into the Pi, log in
5. Robot: placed at the start position, bottom-left corner, facing North
6. Pi, in PuTTY: `cd mdp` then `python3 fake_rpi_client.py <ip>`

---

## 4. Where the output goes (on the Pi)

| Path | Contents |
|---|---|
| `mdp/cmds/<timestamp>_cmds.txt` | The command list for that run, one per line |
| `mdp/cmds/latest.txt` | Copy of the most recent run |
| `mdp/photos/obstacle<id>_<timestamp>.jpg` | Annotated capture for each obstacle |

Copy photos back to the laptop with PowerShell:

```powershell
pscp -r pi@192.168.x.x:/home/pi/mdp/photos C:\Users\YourName\Downloads\
```

---

## 5. Useful extras

These all run on the Pi, in PuTTY.

**Test the plan without the robot** — add `--dry-run` on the Pi. Talks to the server and prints the commands, but never opens the serial port or the camera:

```bash
python3 fake_rpi_client.py 192.168.x.x --dry-run
```

**Replay the last run** without the laptop:

```bash
cd mdp
python3 rpi_stm_conn.py -f cmds/latest.txt
```

**Test a single movement** while calibrating:

```bash
python3 rpi_stm_conn.py FW50
python3 rpi_stm_conn.py RT90
```

---

## 6. If something goes wrong

| Symptom | Cause |
|---|---|
| Pi times out connecting | Firewall still on, or the two machines are on different networks. Try `ping <ip>` from the Pi. |
| Connection refused | Server isn't running, or you're using the wrong IP. |
| Robot doesn't move, commands time out | STM not on `/dev/ttyACM0`, or the firmware doesn't reply `DONE`. Check with `ls /dev/ttyACM0`. |
| `Camera unavailable` at startup | The run continues without capture. Read the traceback above the message — usually a missing model folder or picamera2 in a venv. |
| `[ANDROID] failed` on each SNAP | `/dev/rfcomm0` not bound. Photos are still saved; only the tablet message is lost. |
