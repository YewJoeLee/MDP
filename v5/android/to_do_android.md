# Android team: V5
All original Android project files are byte-for-byte unchanged. This note is the only addition.

- Build and install MDPAndroid using your existing workflow.
- Check TARGET messages on success and "no image found" on an exhausted retry.
- Confirm the robot marker moves backward for the backup; the RPi now uses the calibrated algo/config.py turn models.
- Confirm STOP between commands and frames. Existing protocol does not interrupt an STM movement already in progress.
- No Android implementation work is required for this recovery policy.
