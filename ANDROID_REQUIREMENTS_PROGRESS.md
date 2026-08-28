# Android Remote Controller Requirements and Progress

Last reviewed: 27 August 2026  
Branch reviewed: `appV1`  
Implementation commit reviewed: `e17c11a`

This document translates the supplied MDP material into an Android-focused work tracker. The PDFs remain the authoritative assessment documents. Statuses below describe the current code and the verification evidence available in this repository; an item is not considered assessment-ready until it has been demonstrated with the AMD Tool or the team robot and signed on the checklist.

## Status legend

- **Implemented - evidence pending:** the Android code covers the requirement, but a live Bluetooth/robot demonstration is still required.
- **Implemented - integration pending:** the feature exists, but the team must confirm the protocol, coordinate convention, or hardware behavior.
- **Partial / follow-up needed:** the main feature exists, but a known detail still needs correction or confirmation.
- **Conditional team item:** it comes from the general MDP briefing rather than the ARCM checklist and applies only if Android is used for that team deliverable.

## Source documents reviewed

1. `Android Remote Controller checklist.pdf` - 2 pages; formal ARCM functional specifications C.1-C.10.
2. `MDP ARCM Briefing Slides.pdf` - 14 pages; ARCM purpose, assessment context, examples, and suggested message formats.
3. `MDP briefing.pdf` - 32 pages; overall MDP task, system assessment, task assessments, deadlines, and subsystem responsibilities.

## ARCM checklist audit

| ID | Requirement from the supplied material | Current implementation | Status and remaining evidence |
|---|---|---|---|
| C.1 | Send and receive text strings over a Bluetooth serial link. The AMD Tool may be used to prove bidirectional transfer. | `BluetoothController` uses classic Bluetooth SPP, sends UTF-8 newline-terminated commands, reads newline-delimited incoming messages, and parses `MSG`, `TARGET`, and `ROBOT`. | **Implemented - evidence pending.** Pair with the AMD Tool or robot and demonstrate one Android-to-device message and one device-to-Android message. Confirm that the final team protocol expects newline termination. |
| C.2 | GUI initiates Bluetooth scanning, device selection, and connection. | The connection card provides Scan, a device picker containing paired/discovered devices, and Connect/Disconnect controls. Android 12+ Nearby Devices and older location permissions are declared/requested. | **Implemented - evidence pending.** On the target tablet, verify permission approval, scanning, selection, and connection to the actual SPP device. |
| C.3 | GUI provides interactive robot movement control; manually typing commands in a text box does not count. | A labeled directional control sends `MOVE,F`, `MOVE,B`, `MOVE,L`, `MOVE,R`, plus `STOP`. There is no command-entry text box. | **Implemented - integration pending.** Confirm these command strings and their movement semantics with the robot/RPi team, then demonstrate movement through Bluetooth. |
| C.4 | GUI displays selected remote status/update messages, not the complete raw stream. | Incoming `MSG,...` messages are parsed into a timestamped Robot status list. Unknown raw messages are not displayed verbatim. | **Implemented - evidence pending.** Demonstrate messages such as ready, moving, or target progress from the AMD Tool/robot and confirm the selected display content is acceptable to the supervisor. |
| C.5 | 2D display of the arena with numbered obstacles and the robot location/facing direction. | A custom Canvas draws a bounded 20x20 grid, numbered obstacle blocks, a robot marker, and a directional indicator. Valid app indices are x `0..19` and y `0..19`. | **Implemented - integration pending.** The 20x20 dimensions are now reflected in the app. Confirm the team’s origin, y-axis direction, and obstacle-cell convention before assessment. |
| C.6 | Touch-based obstacle placement and touch-and-drag movement. Dragging outside the map removes it; releasing sends obstacle number and coordinates over Bluetooth. | Tapping an empty cell adds the next `B#` obstacle and sends `ADD,B#,(x,y)`. Dragging an obstacle moves it; invalid/outside coordinates call removal and send `SUB,B#`. | **Implemented - evidence pending.** Test on the tablet that tap, drag, release, collision handling, and dragging beyond the map all behave correctly. Confirm that the robot-side parser accepts the chosen `ADD`/`SUB` format. |
| C.7 | Touch-based annotation of which obstacle face (N/W/E/S) contains the target image. The appearance changes and face plus obstacle coordinate are sent. | Selecting an obstacle exposes touchable N/S/E/W chips. The selected obstacle gets a thick red face line. The app sends `FACE,B#,N,(x,y)` including the coordinate. | **Implemented - integration pending.** The alternative chip interaction is allowed by the checklist, but confirm the robot/RPi team accepts the final field order and coordinate representation. |
| C.8 | App must not hang after temporary Bluetooth loss and should automatically reconnect when the device becomes available again. | Read, connection, and write work use separate executors. EOF/read/write failures update the UI, close the socket, retain the last device, and retry after 3 seconds. | **Implemented - evidence pending.** Deliberately disconnect the AMD Tool/robot, verify the app remains responsive, reconnect the device, and verify the app reconnects and can send again. |
| C.9 | On receiving `TARGET, <Obstacle Number>, <Target ID>`, display the target ID on that obstacle; if a face is supplied, show a thick distinguishing line. | `TARGET,B2,11` changes the obstacle fill and displays `11` in larger white text. `TARGET,B2,11,N` also marks the N face with a thick red line. | **Implemented - evidence pending.** Send both target formats from the AMD Tool and verify the correct obstacle, ID, and optional face are updated. The obstacle must already exist in the local map. |
| C.10 | On receiving `ROBOT, <x>, <y>, <direction>`, update the robot position and direction; direction is N/S/E/W. | `ROBOT,x,y,direction` is parsed into robot state and redraws the marker/directional indicator. | **Implemented - evidence pending.** Send valid messages for all four directions and verify the marker is placed using the agreed coordinate convention. |

## General MDP requirements affecting Android

These points are not additional ARCM checklist rows, but they can affect your Android deliverable or team integration.

| Source requirement | Relevance to this app | Progress |
|---|---|---|
| The robotic system must transmit and receive control signals from a mobile device. | Covered by the Bluetooth transport, movement controls, map commands, and incoming state parser. | **Covered in code; live integration pending.** |
| The evaluation arena is not necessarily a fixed physical location. | The current app uses a 20x20 logical map with x `0..19` and y `0..19`, as specified for this project. The team must still confirm that this logical coordinate map matches the robot/algorithm representation when the physical arena is laid out elsewhere. | **Implemented - integration confirmation needed.** |
| For the image challenge, stitched RAW camera images must be shown as one image in Android or on a PC for verification. | The current Android app has no stitched-image viewer. This is a team-level requirement; it is an Android gap only if the team chooses Android rather than a PC for this verification display. | **Conditional team item - not implemented in Android.** |
| The Android tablet is the team’s wireless remote controller and visualizes the current arena/robot status. | The current UI covers the remote-control and visualization role on the 20x20 map. | **Covered in code; live integration pending.** |

## Mandatory task assessments

Both task assessments must be completed by the team:

| Task | Assessment requirement | Android relevance | Progress |
|---|---|---|---|
| Task 1 - Automatic movement and image recognition (12.5%) | The robot autonomously moves through the arena and recognizes the target images. The general briefing gives a six-minute timeout and requires all images in the run to be identified for full points. | Android should support the team’s start/stop or run-control protocol, display `MSG`, `TARGET`, and `ROBOT` updates, and provide the arena/target state for monitoring. The image-recognition and autonomous movement algorithms belong to the robot/RPi/algorithm subsystems. | **Team integration pending.** Agree on task-start, task-stop, and completion messages. Decide whether stitched RAW images will be shown in Android or on a PC for verification. |
| Task 2 - Fastest car using visual recognition (12.5%) | The robot follows visual left/right instructions at goal obstacles, returns to the car park, and stops. The general briefing gives a three-minute timeout and adds a 10-second penalty for each obstacle hit. | Android is not a replacement for the robot’s visual-recognition/autonomous driving logic. It should remain available for status monitoring, emergency stop/manual testing, and any agreed run-control messages. | **Team integration pending.** Confirm whether Android sends a run command or only monitors the run, and test the emergency stop path before the assessment. |

These task assessments are separate from the ARCM C.1-C.10 checklist. Passing the Android checklist does not by itself demonstrate autonomous image recognition or the fastest-car task.

## Assessment and evidence checklist

The general briefing states that system functionality is a 20% group assessment, the supervisor signs demonstrated checklist items, and contributor names should be recorded for each component. The supplied briefing gives the system checklist deadline as Friday 5:00 pm of Week 7. It also lists an Android-related individual quiz in Week 7, an image-recognition task in Week 8, a fastest-robot task in Week 9, and a maximum-five-minute video due in Week 10.

For the Android contribution, prepare evidence for:

- Bluetooth permissions, scanning, device selection, connection, and bidirectional text transfer.
- Movement commands demonstrated through the AMD Tool or the real robot.
- A selective status message received from the robot.
- At least two obstacles added/moved/removed, including the Bluetooth messages produced.
- One face annotation, its changed appearance, and the outgoing message.
- A `TARGET` update with and without a face.
- `ROBOT` updates for multiple positions and directions.
- A temporary disconnect followed by successful automatic reconnection.
- Your name entered as contributor for the Android checklist items you completed.

## Integration decisions still needed

1. **Movement protocol:** confirm whether `MOVE,F`, `MOVE,B`, `MOVE,L`, `MOVE,R`, and `STOP` match the robot/RPi implementation, or replace them with the team’s agreed strings.
2. **Obstacle protocol:** confirm `ADD,B1,(x,y)` and `SUB,B1` and whether coordinates are zero-based.
3. **Face protocol:** confirm the current `FACE,B1,N,(x,y)` format. The ARCM slide shows `FACE,B2,N`, while the formal checklist additionally requires the obstacle coordinate.
4. **Incoming protocol:** confirm the exact delimiters, newline behavior, and whether `TARGET` face direction is always included.
5. **Map dimensions/convention:** the arena is now configured as 20×20 with valid x/y indices `0..19`; confirm the origin, y-axis direction, and whether obstacles occupy one grid cell.
6. **Image verification:** decide whether the stitched RAW image display belongs in Android or on the PC side.
7. **Bluetooth hardware:** confirm that the target is classic Bluetooth SPP. The current implementation is not a BLE GATT client.

## Overall conclusion

The current app covers the functional shape of all ten ARCM checklist items. No checklist item is clearly absent from the Android code. The map is configured for the required 20x20 arena with indices `0..19`, but the coordinate convention and all Bluetooth message formats still need final team agreement. C.1-C.4 and C.6-C.10 still need live AMD Tool/robot demonstrations. Both mandatory task assessments also require cross-subsystem integration; the Android app can monitor and control the agreed interface, but it does not implement the robot’s autonomous movement or image-recognition algorithms. The stitched RAW image viewer is the only notable Android feature not present, and it is conditional on the team choosing Android for that general MDP verification requirement.
