import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand
import "../components"

Item {
    id: root

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Theme.pageMargin
        spacing: 14

        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            ColumnLayout {
                spacing: 3

                Text {
                    text: "CONSOLE"
                    color: Theme.textPrimary
                    font.family: Theme.displayFont
                    font.pixelSize: 22
                    font.bold: true
                    font.letterSpacing: 2
                }

                Text {
                    text: "Device terminal output and OrbitCommand events"
                    color: Theme.textMuted
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                }
            }

            Item { Layout.fillWidth: true }

            RowLayout {
                spacing: 8

                Text {
                    text: "AUTO-SCROLL"
                    color: Theme.textPrimary
                    font.family: Theme.monoFont
                    font.pixelSize: 10
                    font.letterSpacing: 1.2
                }

                NeonSwitch {
                    id: autoScrollSwitch
                    objectName: "oc.console.autoScrollSwitch"
                    Accessible.name: "Console auto-scroll"
                    checked: true
                }
            }

            NeonButton {
                objectName: "oc.console.clearButton"
                Accessible.name: "Clear console"
                text: "CLEAR"
                filled: false
                onClicked: app.clearLog()
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: Theme.panel
            border.color: Theme.border
            border.width: 1
            radius: Theme.radius

            ConsolePane {
                anchors.fill: parent
                anchors.margins: 6
                model: app.logModel
                autoScroll: autoScrollSwitch.checked
            }
        }
    }
}
