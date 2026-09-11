package com.example.mdpandroid

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import com.example.mdpandroid.ui.ARCMApp
import com.example.mdpandroid.ui.theme.MDPAndroidTheme

class MainActivity : ComponentActivity() {
    private lateinit var bluetoothController: BluetoothController

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        bluetoothController.refreshDevices()
        if (result.values.all { granted -> granted }) {
            bluetoothController.startScan()
            bluetoothController.startServerListening()
        } else {
            bluetoothController.addStatus("Bluetooth permission is required before scanning")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        bluetoothController = BluetoothController(applicationContext)
        setContent {
            MDPAndroidTheme {
                ARCMApp(bluetoothController, ::requestBluetoothPermissions)
            }
        }
    }

    private fun requestBluetoothPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        val missing = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isEmpty()) {
            bluetoothController.refreshDevices()
            bluetoothController.startScan()
            bluetoothController.startServerListening()
        } else {
            permissionLauncher.launch(missing.toTypedArray())
        }
    }

    override fun onDestroy() {
        if (::bluetoothController.isInitialized) bluetoothController.close()
        super.onDestroy()
    }
}
