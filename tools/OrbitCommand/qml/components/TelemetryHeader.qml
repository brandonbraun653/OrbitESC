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
            objectName: "oc.header.targetCombo"
            Accessible.name: "Connection target"
            Accessible.role: Accessible.ComboBox
            Layout.preferredWidth: root.width >= 1320 ? 210 : 175
            Layout.preferredHeight: 36
            model: app.availableTargets
            textRole: "label"
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
                text: portCombo.displayText.length > 0 ? portCombo.displayText : "NO TARGET"
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
                if (index >= 0 && index < app.availableTargets.length)
                    app.selectTarget(app.availableTargets[index].id)
            }

            function syncSelection() {
                for (var i = 0; i < app.availableTargets.length; ++i) {
                    if (app.availableTargets[i].id === app.selectedTargetId) {
                        currentIndex = i
                        return
                    }
                }
                currentIndex = app.availableTargets.length > 0 ? 0 : -1
            }

            Component.onCompleted: syncSelection()
        }

        RowLayout {
            visible: app.simulatorTargetSelected
            Layout.preferredHeight: 36
            spacing: 6

            Text {
                text: "HOST"
                color: Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.2
            }

            TextField {
                id: simulatorHostField
                objectName: "oc.header.simHost"
                Accessible.name: "Simulator host"
                Layout.preferredWidth: 115
                Layout.preferredHeight: 36
                text: app.simulatorHost
                enabled: !app.connected
                selectByMouse: true
                font.family: Theme.monoFont
                font.pixelSize: 11
                color: enabled ? Theme.textPrimary : Theme.textMuted

                background: Rectangle {
                    radius: Theme.radius
                    color: Theme.background
                    border.color: simulatorHostField.activeFocus ? Theme.primary : Theme.border
                    border.width: 1
                }

                onEditingFinished: app.setSimulatorHost(text)
            }

            Text {
                text: "PORT"
                color: Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.2
            }

            SpinBox {
                id: simulatorPortField
                objectName: "oc.header.simPort"
                Accessible.name: "Simulator port"
                Layout.preferredWidth: 84
                Layout.preferredHeight: 36
                from: 1
                to: 65535
                value: app.simulatorPort
                editable: true
                enabled: !app.connected
                font.family: Theme.monoFont
                font.pixelSize: 11
                palette.text: enabled ? Theme.textPrimary : Theme.textMuted
                palette.base: Theme.background
                palette.button: Theme.panel
                palette.mid: Theme.border

                background: Rectangle {
                    radius: Theme.radius
                    color: Theme.background
                    border.color: simulatorPortField.activeFocus ? Theme.primary : Theme.border
                    border.width: 1
                }

                onValueModified: app.setSimulatorPort(value)
            }
        }

        NeonButton {
            objectName: "oc.header.scanButton"
            Accessible.name: "Scan for connection targets"
            text: "SCAN"
            filled: false
            enabled: !app.connected
            onClicked: app.refreshTargets()
        }

        NeonButton {
            objectName: "oc.header.connectButton"
            Accessible.name: app.connected ? "Disconnect from device" : "Connect to device"
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
                objectName: "oc.header.systemMode"
                text: app.systemMode
                color: app.systemMode === "FAULT" ? Theme.fault : Theme.primary
                font.family: Theme.displayFont
                font.pixelSize: 14
                font.bold: true
                font.letterSpacing: 1.6
            }
        }

        StatusPill {
            objectName: "oc.header.statusPill"
            Accessible.name: "Link status " + text
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
                objectName: "oc.header.tickAge"
                text: app.tickAgeMs >= 0 ? app.tickAgeMs + " ms" : "—"
                color: app.tickAgeMs >= 0 && app.tickAgeMs < 1000 ? Theme.success : Theme.textMuted
                font.family: Theme.monoFont
                font.pixelSize: 12
            }
        }

        NeonButton {
            objectName: "oc.header.estopButton"
            Accessible.name: "Emergency stop"
            text: "E-STOP"
            danger: true
            enabled: app.online
            onClicked: app.emergencyStop()
        }
    }

    Connections {
        target: app
        function onAvailableTargetsChanged() { portCombo.syncSelection() }
        function onSelectedTargetIdChanged() { portCombo.syncSelection() }
    }
}
