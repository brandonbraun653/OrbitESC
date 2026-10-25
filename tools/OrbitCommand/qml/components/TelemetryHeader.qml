import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand

Rectangle {
    id: root

    color: Theme.panel
    border.color: Theme.border
    border.width: 1

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 22
        anchors.rightMargin: 22
        spacing: 12

        ColumnLayout {
            Layout.preferredWidth: root.width >= 1320 ? 250 : 180
            spacing: 2

            Text {
                text: "ORBITCOMMAND"
                color: Theme.textPrimary
                font.family: Theme.displayFont
                font.pixelSize: 18
                font.bold: true
                font.letterSpacing: 2.4
            }

            Text {
                text: "BLDC CONTROL CONSOLE"
                color: Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.4
            }
        }

        Rectangle {
            Layout.preferredWidth: 1
            Layout.preferredHeight: 38
            color: Theme.border
        }

        ComboBox {
            id: portCombo
            Layout.preferredWidth: root.width >= 1320 ? 230 : 190
            Layout.preferredHeight: 36
            model: app.availablePorts
            textRole: "name"
            enabled: !app.connected
            font.family: Theme.monoFont
            font.pixelSize: 11

            background: Rectangle {
                radius: Theme.radius
                color: Theme.background
                border.color: portCombo.activeFocus ? Theme.primary : Theme.border
                border.width: 1
            }

            contentItem: Text {
                leftPadding: 10
                text: portCombo.displayText.length > 0 ? portCombo.displayText : "NO SERIAL TARGET"
                color: portCombo.enabled ? Theme.textPrimary : Theme.textMuted
                font: portCombo.font
                verticalAlignment: Text.AlignVCenter
                elide: Text.ElideRight
            }

            indicator: Text {
                x: portCombo.width - width - 10
                anchors.verticalCenter: parent.verticalCenter
                text: "▾"
                color: Theme.primary
                font.pixelSize: 12
            }

            onActivated: function(index) {
                if (index >= 0 && index < app.availablePorts.length)
                    app.selectPort(app.availablePorts[index].name)
            }

            function syncSelection() {
                for (var i = 0; i < app.availablePorts.length; ++i) {
                    if (app.availablePorts[i].name === app.selectedPort) {
                        currentIndex = i
                        return
                    }
                }
                currentIndex = app.availablePorts.length > 0 ? 0 : -1
            }

            Component.onCompleted: syncSelection()
        }

        NeonButton {
            text: "SCAN"
            filled: false
            enabled: !app.connected
            onClicked: app.refreshPorts()
        }

        NeonButton {
            text: app.connected ? "DISCONNECT" : "CONNECT"
            onClicked: app.toggleConnection()
        }

        Item { Layout.fillWidth: true }

        ColumnLayout {
            visible: root.width >= 1320
            spacing: 2
            Layout.alignment: Qt.AlignVCenter

            Text {
                text: "SYSTEM MODE"
                color: Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.2
            }

            Text {
                text: app.systemMode
                color: app.systemMode === "FAULT" ? Theme.fault : Theme.primary
                font.family: Theme.displayFont
                font.pixelSize: 14
                font.bold: true
                font.letterSpacing: 1.6
            }
        }

        StatusPill {
            text: app.connectionState
            tone: app.online ? "ok" : (app.connected ? "warn" : "idle")
        }

        ColumnLayout {
            visible: root.width >= 1320
            spacing: 2

            Text {
                text: "TICK AGE"
                color: Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.2
            }

            Text {
                text: app.tickAgeMs >= 0 ? app.tickAgeMs + " ms" : "—"
                color: app.tickAgeMs >= 0 && app.tickAgeMs < 1000 ? Theme.success : Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 12
            }
        }

        NeonButton {
            text: "E-STOP"
            danger: true
            enabled: app.online
            onClicked: app.emergencyStop()
        }
    }

    Connections {
        target: app
        function onAvailablePortsChanged() { portCombo.syncSelection() }
        function onSelectedPortChanged() { portCombo.syncSelection() }
    }
}
