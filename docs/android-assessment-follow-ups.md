# Android assessment follow-ups

## Deferred by the user: selective status display (C.4)

- [ ] Keep curated robot status separate from raw Bluetooth diagnostics. The current combined Robot activity panel remains as requested on 1 October 2026.
- [ ] Demonstrate that unrelated telemetry does not appear in the selective status display.

## Integration verification before assessment

- [ ] Verify the fixed existing Pi format: outgoing commands have no added newline; every incoming message must end with a newline or carriage return, including `MSG`, `TARGET`, and `ROBOT`. The supplied Pi client already terminates `TARGET` lines.
- [ ] If the team changes this contract later, update Android and the Pi together. There is no connection-format selector or saved format override.
- [ ] Verify **Send Arena** in both tabs receives only the existing obstacle tuple list, with no ROBOT prefix and no added newline. Both tabs share the same button implementation and send callback. Verify the Pi treats the layout as replacement state, including deleted obstacles and an empty arena list. Android rejects arena sends if any obstacle has no face.
- [ ] Verify task start tokens `beginExplore` / `beginFastest` and movement tokens `f` / `r` / `tl` / `tr` with the actual robot receiver.
- [ ] Configure and verify the Pi sends the agreed bare completion record `taskComplete\n` when either task has finished. Android ends the active task when this complete record arrives; `MSG,taskComplete` is a status message and does not end the task. Do not infer completion from a target result or a Bluetooth disconnect.
- [ ] Run reconnect without restarting the Pi, rotation with a live connection, map editing while a task is running, and a complete assessment run with the final APK.
- [ ] Run the arena gesture instrumentation tests on the tablet. They are compiled here but require a connected device to execute.

## Task readiness and delivery

The robot stops automatically; there is no STOP button. Both task buttons require a connection (or Demo mode), at least one obstacle, a face for each obstacle, a valid robot footprint, and a successful send of the current obstacle layout in the current connection. Pending arena or drive writes also block task starts. Reconnecting requires another arena send.

Both Send Arena buttons reject obstacles without a face before recording a demo send or writing to Bluetooth, and list the affected obstacle IDs in Robot activity. An empty arena can still be sent to clear a previous layout.

Arena also has Task 1 Explore directly below Send arena. Arena and Controls use the same task-button component, readiness checks and controller command callback. Task 2 Fastest path remains in Controls. Both starts require the current arena to have been sent successfully; neither sends an arena automatically.

Obstacle and robot settings remain editable during a task, as requested on 2 October 2026. Manual driving, arena sends and repeated task starts remain unavailable while a task is active. TARGET and ROBOT telemetry continue to update the display. Active task tracking survives rotation and Activity saved-state restoration. Demo runs have a local Finish demo task action.

Receiving `taskComplete` followed by a newline or carriage return ends the active task, including a task restored with uncertain delivery. Obstacle layout, recognized targets, and the latest robot pose remain visible. The arena must be sent again before the next task. Duplicate completion records received while no task is active do not invalidate the next arena's readiness.

Each write batch has a two-second deadline. A stalled write causes socket closure and an explicit delivery-unknown warning. Successful transmission means the bytes were written, not that the robot acknowledged or executed them. No commands are replayed automatically after reconnect.

## Outgoing messages

Obstacle additions, moves, deletions, face changes, face clearing, and robot start/facing edits change only the local arena. They do not send Bluetooth messages. Use **Send Arena** to publish the complete obstacle layout. The locally configured robot pose is not transmitted. Other outgoing messages are manual drive commands and task start commands.

## Saved activity history

Robot activity is shown in Arena and Controls. Overview shows mission and connection information without the activity panel. The updated app saves status and received messages together on the device with full timestamps. There is no automatic entry-count limit on saved history. The activity panel loads older/newer entries as you scroll and keeps a small display window in memory. Browsing older history preserves your place while new logs arrive; Jump to latest resumes following incoming messages.

Short entries show the source, HH:mm:ss time and message on one line when they fit the available width. Longer entries show the message below the source and time, using the full width and wrapping up to three lines in the compact arena panel. Tap an entry for its full, selectable text and complete date/time, including milliseconds. Export logs writes the entire saved history with full timestamps in chronological order to a text file selected through Android's document picker. Clear history asks for confirmation and removes all saved entries. History survives app/process restarts. Entries discarded by older APKs cannot be recovered.

## Saved map and reset

Obstacles, faces, recognized targets and the robot pose are saved locally after changes and restored on app restart. Selection, Bluetooth connections and send readiness are not saved. A cold launch does not resume an old task automatically. An Activity bundle cannot replace a more recent saved map.

Reset map is available beside the arena in landscape and below it in portrait. After confirmation it removes all obstacles and targets, resets the robot to (1, 1), facing North, closes settings and saves the empty map. It preserves logs and any active task, sends no Bluetooth command and invalidates arena readiness. Reset does not stop the physical robot.

## Button verification on 2 October 2026

App-side tests captured the controller's queued output: the arena tuple list is written once with no ROBOT prefix or newline; Task 1 writes `beginExplore`; Task 2 writes `beginFastest`. Completion clears task state and requires a fresh arena send. Missing faces, pending arena writes and active tasks block inappropriate sends/starts. A failed task write leaves delivery uncertain and clears connection readiness. Compose tests exercise both Arena layouts and the Controls buttons. The complete run passed 57 JVM tests and 37 emulator tests, with clean lint and debug APK builds.

These capture tests substitute the external output stream; physical Bluetooth transmission and execution by the Pi remain part of the live integration checks above. The two previously supplied Python files do not contain the receiver's task-token dispatch, so they cannot establish that the Pi implements both task commands.
