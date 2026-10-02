# Android assessment follow-ups

## Deferred by the user: selective status display (C.4)

- [ ] Keep curated robot status separate from raw Bluetooth diagnostics. The current combined Robot activity panel remains as requested on 1 October 2026.
- [ ] Demonstrate that unrelated telemetry does not appear in the selective status display.

## Integration verification before assessment

- [ ] Verify the fixed existing Pi format: outgoing commands have no added newline; every incoming message must end with a newline or carriage return, including `MSG`, `TARGET`, and `ROBOT`. The supplied Pi client already terminates `TARGET` lines.
- [ ] If the team changes this contract later, update Android and the Pi together. There is no connection-format selector or saved format override.
- [ ] Verify **Send Arena** in both tabs receives only the existing obstacle tuple list, with no ROBOT prefix and no added newline. Both tabs share the same button implementation and send callback. Verify the Pi treats the layout as replacement state, including deleted obstacles and empty face strings.
- [ ] Verify task start tokens `beginExplore` / `beginFastest`, movement tokens `f` / `r` / `tl` / `tr`, and `STOP` with the actual robot receiver.
- [ ] Run reconnect without restarting the Pi, rotation with a live connection, map editing while receiving telemetry, and a complete assessment run with the final APK.
- [ ] Run the arena gesture instrumentation tests on the tablet. They are compiled here but require a connected device to execute.

## Stop and delivery semantics

Stop cancels pending commands and runs after the active write. Each write batch has a two-second deadline. A stalled write causes socket closure and an explicit delivery-unknown warning; an unavailable Bluetooth link cannot guarantee a physical stop. Successful transmission means the bytes were written, not that the robot acknowledged or executed them. No commands are replayed automatically after reconnect.

## Outgoing messages

Obstacle additions, moves, deletions, face changes, face clearing, and robot start/facing edits change only the local arena. They do not send Bluetooth messages. Use **Send Arena** to publish the complete obstacle layout. The locally configured robot pose is not transmitted. Other outgoing messages are manual drive commands, STOP, and task start commands.
