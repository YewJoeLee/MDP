# Android assessment follow-ups

## Deferred by the user: selective status display (C.4)

- [ ] Keep curated robot status separate from raw Bluetooth diagnostics. The current combined Robot activity panel remains as requested on 1 October 2026.
- [ ] Demonstrate that unrelated telemetry does not appear in the selective status display.

## Integration verification before assessment

- [ ] Verify the fixed existing Pi format: outgoing commands have no added newline; every incoming message must end with a newline or carriage return, including `MSG`, `TARGET`, and `ROBOT`. The supplied Pi client already terminates `TARGET` lines.
- [ ] If the team changes this contract later, update Android and the Pi together. There is no connection-format selector or saved format override.
- [ ] Verify **Send Arena** in both tabs receives only the existing obstacle tuple list, with no ROBOT prefix and no added newline. Both tabs share the same button implementation and send callback. Verify the Pi treats the layout as replacement state, including deleted obstacles and an empty arena list. Android rejects arena sends if any obstacle has no face.
- [ ] Verify task start tokens `beginExplore` / `beginFastest` and movement tokens `f` / `r` / `tl` / `tr` with the actual robot receiver.
- [ ] Configure and verify the Pi sends the agreed bare completion record `taskComplete\n` when either task has finished. Android unlocks the map when this complete record arrives; `MSG,taskComplete` is a status message and does not unlock it. Do not infer completion from a target result or a Bluetooth disconnect.
- [ ] Run reconnect without restarting the Pi, rotation with a live connection, map locking while receiving telemetry, and a complete assessment run with the final APK.
- [ ] Run the arena gesture instrumentation tests on the tablet. They are compiled here but require a connected device to execute.

## Task readiness and delivery

The robot stops automatically; there is no STOP button. Both task buttons require a connection (or Demo mode), at least one obstacle, a face for each obstacle, a valid robot footprint, and a successful send of the current obstacle layout in the current connection. Pending arena or drive writes also block task starts. Reconnecting requires another arena send.

Both Send Arena buttons reject obstacles without a face before recording a demo send or writing to Bluetooth, and list the affected obstacle IDs in Robot activity. An empty arena can still be sent to clear a previous layout.

Starting a task locks obstacle and robot-settings edits, manual driving, arena sends, and repeated starts. TARGET and ROBOT telemetry continue to update the display. The lock survives rotation and saved-state restoration. An uncertain task-start delivery keeps the map locked. Demo runs have a local Finish demo task action.

Receiving `taskComplete` followed by a newline or carriage return ends the active task and unlocks the map, including a task restored with uncertain delivery. Obstacle layout, recognized targets, and the latest robot pose remain visible. The arena must be sent again before the next task. Duplicate completion records received while no task is active do not invalidate the next arena's readiness.

Each write batch has a two-second deadline. A stalled write causes socket closure and an explicit delivery-unknown warning. Successful transmission means the bytes were written, not that the robot acknowledged or executed them. No commands are replayed automatically after reconnect.

## Outgoing messages

Obstacle additions, moves, deletions, face changes, face clearing, and robot start/facing edits change only the local arena. They do not send Bluetooth messages. Use **Send Arena** to publish the complete obstacle layout. The locally configured robot pose is not transmitted. Other outgoing messages are manual drive commands and task start commands.
