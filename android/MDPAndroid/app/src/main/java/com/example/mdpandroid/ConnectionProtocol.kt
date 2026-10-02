package com.example.mdpandroid

/** Current Pi contract: outgoing command text has no added terminator. */
object ConnectionProtocol {
    fun payload(command: String): String = command.trim()
}

internal fun AppState.connectedTo(device: BluetoothDeviceInfo): AppState = copy(
    demoMode = false,
    selectedDevice = device,
    selectedDeviceName = device.name ?: device.address,
    connected = true,
    connectedAddress = device.address,
    connectionStatus = "Connected",
    connectionDetail = RobotMessages.connectedTo(device.name ?: device.address)
)
