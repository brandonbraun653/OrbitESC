import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
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
                    text: "PARAMETERS"
                    color: Theme.textPrimary
                    font.family: Theme.displayFont
                    font.pixelSize: 22
                    font.bold: true
                    font.letterSpacing: 2
                }

                Text {
                    text: "Firmware parameter table, read/write with explicit Apply"
                    color: Theme.textMuted
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                }
            }

            Item { Layout.fillWidth: true }

            StatusPill {
                text: app.dirtyCount + " DIRTY"
                tone: app.dirtyCount > 0 ? "warn" : "idle"
            }

            BusyIndicator {
                running: app.busy
                visible: app.busy
                implicitWidth: 26
                implicitHeight: 26
            }

            Text {
                visible: app.busy
                Layout.maximumWidth: 320
                text: app.progressText
                color: Theme.primary
                font.family: Theme.monoFont
                font.pixelSize: 10
                elide: Text.ElideRight
            }

            NeonButton {
                text: "REFRESH"
                filled: false
                enabled: app.online && !app.busy
                onClicked: app.refreshParameters()
            }

            NeonButton {
                text: "APPLY"
                enabled: app.online && !app.busy && app.dirtyCount > 0
                onClicked: app.applyParameters()
            }

            NeonButton {
                text: "LOAD"
                filled: false
                enabled: !app.busy
                onClicked: loadDialog.open()
            }

            NeonButton {
                text: "SAVE"
                filled: false
                onClicked: saveDialog.open()
            }
        }

        TabBar {
            id: tabs
            Layout.fillWidth: true
            spacing: 4

            background: Rectangle {
                color: "transparent"
            }

            Repeater {
                model: [ "Information", "System Behavior", "Motor Control", "Motor Attributes" ]

                TabButton {
                    text: modelData.toUpperCase()
                    font.family: Theme.monoFont
                    font.pixelSize: 10
                    font.letterSpacing: 1

                    background: Rectangle {
                        radius: 6
                        color: tabs.currentIndex === index ? Qt.rgba(0, 0.9, 1, 0.08) : "transparent"
                        border.color: tabs.currentIndex === index ? Theme.primary : Theme.border
                        border.width: 1
                    }

                    contentItem: Text {
                        text: modelData.toUpperCase()
                        color: tabs.currentIndex === index ? Theme.primary : Theme.textMuted
                        font.family: Theme.monoFont
                        font.pixelSize: 10
                        font.letterSpacing: 1
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabs.currentIndex

            ParameterGroupView {
                model: app.informationModel
                groupTitle: "Information"
            }

            ParameterGroupView {
                model: app.systemModel
                groupTitle: "System Behavior"
            }

            ParameterGroupView {
                model: app.motorControlModel
                groupTitle: "Motor Control"
            }

            ParameterGroupView {
                model: app.motorAttributesModel
                groupTitle: "Motor Attributes"
            }
        }
    }

    FileDialog {
        id: saveDialog
        title: "Save Parameters"
        fileMode: FileDialog.SaveFile
        nameFilters: [ "OrbitCommand parameters (*.json)", "All files (*)" ]
        defaultSuffix: "json"
        onAccepted: app.saveParameters(selectedFile)
    }

    FileDialog {
        id: loadDialog
        title: "Load Parameters"
        fileMode: FileDialog.OpenFile
        nameFilters: [ "OrbitCommand parameters (*.json)", "All files (*)" ]
        onAccepted: app.loadParameters(selectedFile)
    }
}
