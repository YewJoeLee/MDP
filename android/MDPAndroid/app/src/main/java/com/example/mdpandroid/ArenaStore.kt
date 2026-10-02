package com.example.mdpandroid

import android.annotation.SuppressLint
import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import java.util.concurrent.Executors
import java.util.concurrent.Future

/** Stores map data only. Bluetooth delivery and active tasks are session state. */
class ArenaStore(context: Context, name: String = "saved-arena") {
    private val preferences = context.getSharedPreferences(name, Context.MODE_PRIVATE)
    private val worker = Executors.newSingleThreadExecutor()
    private var closeCompletion: Future<*>? = null
    var loadError: String? = null
        private set

    fun load(): ArenaSnapshot? {
        val saved = preferences.getString("map", null) ?: return null
        return try {
            val root = JSONObject(saved)
            require(root.getInt("version") == 1)
            val items = root.getJSONArray("obstacles")
            require(items.length() <= MAP_COLUMNS * MAP_ROWS)
            val obstacles = List(items.length()) { index ->
                val item = items.getJSONObject(index)
                val id = item.getString("id")
                val x = item.getInt("x")
                val y = item.getInt("y")
                require(id.isNotBlank() && x in 0 until MAP_COLUMNS && y in 0 until MAP_ROWS)
                Obstacle(id, x, y,
                    if (item.isNull("target")) null else item.getString("target"),
                    if (item.isNull("face")) null else Face.valueOf(item.getString("face")))
            }
            require(obstacles.distinctBy { it.id }.size == obstacles.size)
            require(obstacles.distinctBy { it.x to it.y }.size == obstacles.size)
            val pose = root.getJSONObject("robot")
            val robot = RobotState(pose.getInt("x"), pose.getInt("y"), Face.valueOf(pose.getString("face")))
            require(clampRobotCenter(robot.x, robot.y) == GridPoint(robot.x, robot.y))
            ArenaSnapshot(obstacles, null, robot)
        } catch (_: Exception) {
            loadError = "Saved map could not be read. Please set the map again."
            null
        }
    }

    @SuppressLint("UseKtx") // The KTX edit helper discards commit's failure result.
    @Synchronized fun save(snapshot: ArenaSnapshot, onError: () -> Unit) {
        if (closeCompletion != null) return
        // Update SharedPreferences immediately; Android queues the disk write off the UI thread.
        // This also lets a replacement controller see the newest map before the write finishes.
        try {
            val obstacles = JSONArray()
            snapshot.obstacles.forEach { obstacle ->
                obstacles.put(JSONObject().put("id", obstacle.id).put("x", obstacle.x).put("y", obstacle.y)
                    .put("target", obstacle.targetId ?: JSONObject.NULL)
                    .put("face", obstacle.targetFace?.name ?: JSONObject.NULL))
            }
            val robot = JSONObject().put("x", snapshot.robot.x).put("y", snapshot.robot.y)
                .put("face", snapshot.robot.direction.name)
            val json = JSONObject().put("version", 1).put("obstacles", obstacles).put("robot", robot).toString()
            preferences.edit().putString("map", json).apply()
            worker.execute {
                // Persist a revision so commit performs a write and reports disk failure.
                // It includes the current map without replacing a newer edit's value.
                val saved = runCatching {
                    preferences.edit().putLong("revision", preferences.getLong("revision", 0) + 1).commit()
                }.getOrDefault(false)
                if (!saved) onError()
            }
        } catch (_: Exception) { onError() }
    }

    /** Awaitable in tests; controller shutdown flushes queued writes without blocking the UI. */
    @Synchronized fun close(): Future<*> {
        closeCompletion?.let { return it }
        return worker.submit {}.also {
            closeCompletion = it
            worker.shutdown()
        }
    }
}
