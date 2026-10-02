# Persistent arena and editable runs

User request: allow map editing during a run, retain the map after restart, and add Reset map.

Scope: work directly on appV1; preserve the existing activity-history changes. Save obstacles, optional faces and recognized targets, and robot pose. Keep connection/send readiness session-only. Keep duplicate-start, manual-drive and arena-send task guards. Reset is local, confirmation-protected, saves an empty map and the default (1, 1, North) robot; preserve logs and active task tracking.

Implementation:
1. Change controller and gesture regression checks to allow editing during tasks.
2. Add private, versioned JSON map storage and controller restoration/save hooks. Prefer saved map over stale Activity bundles.
3. Add Reset map to both layouts, close settings on reset, and prevent pre-reset send/drive callbacks from overwriting reset state.
4. Verify unit tests, emulator persistence/reset/gesture tests, lint, APK builds and full-height landscape layout.

Verification completed: 57 JVM tests and 31 emulator tests passed; lint reported no issues; debug and instrumentation APK builds succeeded. Emulator UI checks confirmed the complete landscape map and Reset map button fit, an obstacle/face and moved robot marker survived a full app restart, and a confirmed reset saved an empty map with the default robot pose after another restart. Changes remain uncommitted on appV1.
