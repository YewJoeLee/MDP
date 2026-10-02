package com.example.mdpandroid

import android.content.Context
import androidx.lifecycle.ViewModel

/** Retains the Bluetooth session and arena across Activity configuration changes. */
class RobotSessionViewModel : ViewModel() {
    private var retainedController: BluetoothController? = null

    fun controller(context: Context, savedArena: ArenaSnapshot?): BluetoothController {
        retainedController?.let { return it }
        return BluetoothController(context.applicationContext).also {
            savedArena?.let(it::restoreArena)
            retainedController = it
        }
    }

    override fun onCleared() {
        retainedController?.close()
        retainedController = null
    }
}
