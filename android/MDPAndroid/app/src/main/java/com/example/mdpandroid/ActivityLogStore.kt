package com.example.mdpandroid

import android.content.ContentValues
import android.content.Context
import android.database.sqlite.SQLiteDatabase
import android.database.sqlite.SQLiteOpenHelper
import java.io.Closeable
import java.io.Writer

/** Full history lives in SQLite; callers use the history worker for all disk operations. */
internal class ActivityLogStore(context: Context, name: String = "robot-activity.db") :
    SQLiteOpenHelper(context.applicationContext, name, null, 1), Closeable {
    override fun onCreate(db: SQLiteDatabase) {
        db.execSQL("CREATE TABLE entries (_id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp TEXT NOT NULL, source TEXT NOT NULL, text TEXT NOT NULL)")
    }

    override fun onUpgrade(db: SQLiteDatabase, oldVersion: Int, newVersion: Int) = Unit

    fun append(source: ActivitySource, timestamp: String, text: String): ActivityLogEntry {
        val values = ContentValues().apply {
            put("timestamp", timestamp)
            put("source", source.name)
            put("text", text)
        }
        val id = writableDatabase.insertOrThrow("entries", null, values)
        return ActivityLogEntry(StatusMessage(timestamp, text, id), source)
    }

    fun count(): Long = readableDatabase.rawQuery("SELECT COUNT(*) FROM entries", null).use {
        it.moveToFirst()
        it.getLong(0)
    }

    fun latest(limit: Int): List<ActivityLogEntry> = query(null, null, "_id DESC", limit)
    fun olderThan(id: Long, limit: Int): List<ActivityLogEntry> = query("_id < ?", arrayOf(id.toString()), "_id DESC", limit)
    fun newerThan(id: Long, limit: Int): List<ActivityLogEntry> = query("_id > ?", arrayOf(id.toString()), "_id ASC", limit).asReversed()
    fun hasBefore(id: Long): Boolean = exists("_id < ?", id)
    fun hasAfter(id: Long): Boolean = exists("_id > ?", id)

    fun clear() { writableDatabase.delete("entries", null, null) }

    fun exportTo(writer: Writer) {
        readableDatabase.query("entries", null, null, null, null, null, "_id ASC").use { cursor ->
            while (cursor.moveToNext()) {
                val entry = readEntry(cursor)
                val label = if (entry.source == ActivitySource.STATUS) "Status" else "Received"
                writer.write("${entry.message.time} $label: ${entry.message.text}\n")
            }
        }
        writer.flush()
    }

    private fun query(selection: String?, args: Array<String>?, sort: String, limit: Int): List<ActivityLogEntry> {
        require(limit > 0)
        return readableDatabase.query("entries", null, selection, args, null, null, sort, limit.toString()).use { cursor ->
            buildList { while (cursor.moveToNext()) add(readEntry(cursor)) }
        }
    }

    private fun exists(selection: String, id: Long): Boolean =
        readableDatabase.query("entries", arrayOf("_id"), selection, arrayOf(id.toString()), null, null, null, "1").use { it.moveToFirst() }

    private fun readEntry(cursor: android.database.Cursor): ActivityLogEntry = ActivityLogEntry(
        StatusMessage(cursor.getString(cursor.getColumnIndexOrThrow("timestamp")),
            cursor.getString(cursor.getColumnIndexOrThrow("text")), cursor.getLong(cursor.getColumnIndexOrThrow("_id"))),
        ActivitySource.valueOf(cursor.getString(cursor.getColumnIndexOrThrow("source")))
    )
}
