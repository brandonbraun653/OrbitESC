import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand
import "../components"

Item {
    id: root

    function fmt(value, digits) {
        return value >= 0 ? Number(value).toFixed(digits) : "N/A"
    }

    ScrollView {
        id: scroll
        anchors.fill: parent
        clip: true
        contentWidth: availableWidth

        ColumnLayout {
            width: scroll.availableWidth
            spacing: 18

            Item { Layout.preferredHeight: 6 }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.leftMargin: Theme.pageMargin
                Layout.rightMargin: Theme.pageMargin
                spacing: 4

                Text {
                    text: "SYSTEM DASHBOARD"
                    color: Theme.textPrimary
                    font.family: Theme.displayFont
                    font.pixelSize: 22
                    font.bold: true
                    font.letterSpacing: 2
                }

                Text {
                    text: "Live device state, link health, and protocol-backed telemetry"
                    color: Theme.textMuted
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                }
            }

            GridLayout {
                Layout.fillWidth: true
                Layout.leftMargin: Theme.pageMargin
                Layout.rightMargin: Theme.pageMargin
                columns: 3
                columnSpacing: 14
                rowSpacing: 14

                MetricTile {
                    objectName: "oc.dashboard.systemMode"
                    Accessible.name: "System Mode"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "System Mode"
                    value: app.systemMode
                    valueColor: app.systemMode === "FAULT" ? Theme.fault : Theme.primary
                    detail: app.online ? "Motor control state" : "Link unavailable"
                }

                MetricTile {
                    objectName: "oc.dashboard.linkState"
                    Accessible.name: "Link State"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "Link State"
                    value: app.connectionState.toUpperCase()
                    valueColor: app.online ? Theme.success : (app.connected ? Theme.warning : Theme.textMuted)
                    detail: app.tickAgeMs >= 0 ? app.tickAgeMs + " ms since last tick" : "No tick received"
                }

                MetricTile {
                    objectName: "oc.dashboard.busVoltage"
                    Accessible.name: "DC Bus Voltage"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "DC Bus Voltage"
                    value: fmt(app.busVoltage, 2)
                    unit: "V"
                    valueColor: Theme.textPrimary
                    detail: "ADC_SYSTEM_VOLTAGES.v_dc_link"
                }

                MetricTile {
                    objectName: "oc.dashboard.tempSensor"
                    Accessible.name: "Temperature Sensor"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "Temperature Sensor"
                    value: fmt(app.tempSensorVoltage, 3)
                    unit: "V"
                    valueColor: Theme.textPrimary
                    detail: "Sensor voltage, not Celsius"
                }

                MetricTile {
                    objectName: "oc.dashboard.phaseRms"
                    Accessible.name: "Phase Current RMS"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "Phase Current RMS"
                    value: fmt(app.phaseCurrentRms, 3)
                    unit: "A"
                    valueColor: app.streamEnabled ? Theme.primary : Theme.textMuted
                    detail: app.streamEnabled ? "Last 1 second" : "Enable phase-current stream"
                }

                MetricTile {
                    objectName: "oc.dashboard.phasePeak"
                    Accessible.name: "Phase Current Peak"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 220
                    label: "Phase Current Peak"
                    value: fmt(app.phaseCurrentPeak, 3)
                    unit: "A"
                    valueColor: app.streamEnabled ? Theme.secondary : Theme.textMuted
                    detail: app.streamEnabled ? "Last 1 second" : "Enable phase-current stream"
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.leftMargin: Theme.pageMargin
                Layout.rightMargin: Theme.pageMargin
                spacing: 14

                NeonCard {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 210
                    title: "Not Available In Current Protocol"

                    Text {
                        Layout.fillWidth: true
                        text: "The Python dashboard placeholders for fault code, average current, and estimated RPM are not backed by a dashboard message. OrbitCommand does not fabricate them."
                        color: Theme.textMuted
                        font.family: Theme.monoFont
                        font.pixelSize: 11
                        wrapMode: Text.Wrap
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        columns: 3
                        columnSpacing: 24

                        Text { text: "FAULT CODE"; color: Theme.textDim; font.family: Theme.monoFont; font.pixelSize: 9 }
                        Text { text: "AVERAGE CURRENT"; color: Theme.textDim; font.family: Theme.monoFont; font.pixelSize: 9 }
                        Text { text: "ESTIMATED RPM"; color: Theme.textDim; font.family: Theme.monoFont; font.pixelSize: 9 }
                        Text { text: "N/A"; color: Theme.textMuted; font.family: Theme.monoFont; font.pixelSize: 13 }
                        Text { text: "N/A"; color: Theme.textMuted; font.family: Theme.monoFont; font.pixelSize: 13 }
                        Text { text: "N/A"; color: Theme.textMuted; font.family: Theme.monoFont; font.pixelSize: 13 }
                    }
                }

                NeonCard {
                    Layout.preferredWidth: 340
                    Layout.preferredHeight: 210
                    title: "Device Actions"

                    Text {
                        Layout.fillWidth: true
                        text: "Emergency stop is always available in the header. A system reset reboots the controller and briefly drops the link."
                        color: Theme.textMuted
                        font.family: Theme.monoFont
                        font.pixelSize: 11
                        wrapMode: Text.Wrap
                    }

                    Item { Layout.fillHeight: true }

                    NeonButton {
                        objectName: "oc.dashboard.resetButton"
                        Accessible.name: "Command reset"
                        Layout.fillWidth: true
                        text: "COMMAND RESET"
                        enabled: app.online
                        onClicked: resetDialog.open()
                    }

                    NeonButton {
                        objectName: "oc.dashboard.clearLogButton"
                        Accessible.name: "Clear log"
                        Layout.fillWidth: true
                        text: "CLEAR LOG"
                        filled: false
                        onClicked: app.clearLog()
                    }
                }
            }

            Item { Layout.preferredHeight: 18 }
        }
    }

    ConfirmDialog {
        id: resetDialog
        anchors.centerIn: Overlay.overlay
        message: "Reset the OrbitESC controller? The device will reboot and the serial link will drop temporarily."
        confirmText: "RESET"
        onConfirmed: app.resetSystem()
    }
}
