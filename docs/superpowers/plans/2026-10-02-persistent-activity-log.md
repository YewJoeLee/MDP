# Persistent activity log implementation plan

User approved the saved-history design in chat: retain all entries on the device, load older entries as needed, clear/export history, and expand shortened text. Execute directly on appV1; no worktree or unrelated changes.

## Design

Use Android's built-in SQLite storage, with no new dependencies. Save full timestamps, source, text, and a monotonically increasing row ID. A single background worker orders writes, reads, clear, and streaming export. Keep at most 300 displayed entries; page by row ID in either direction. Follow incoming entries only while viewing the latest logs. Disk history has no automatic entry cap; clearing is explicit.

Keep existing recent status/raw lists for controller diagnostics, but render saved history in every Robot activity panel. Use a shared history model owned by the Bluetooth controller and closed with its session. Export through Android's document picker. Show complete message text in a scrollable dialog.

## Implementation and checks

- [x] Add real SQLite regression tests for retention beyond the old limits, reopen, ordered paging, full-text export, and clear.
- [x] Implement ActivityLogStore and an asynchronous ActivityLogHistory model with generation checks against stale clear/load callbacks.
- [x] Route both status and raw entries into saved history without blocking Bluetooth callbacks.
- [x] Add paging, Latest, Clear, Export, and full-message dialogs to Robot activity in all tabs.
- [x] Check browsing during incoming logs, reload after restart, clear during queued writes, and export failure feedback.
- [x] Run JVM tests, lint, APK builds, and emulator tests. Inspect the landscape layout to ensure the log actions fit without shrinking the map.

## Constraints

Preserve existing Pi payloads, task checks, arena sizing, and the deferred C.4 follow-up. Unrelated IDE/audit/doc files remain untouched. New implementation changes remain uncommitted until requested.
