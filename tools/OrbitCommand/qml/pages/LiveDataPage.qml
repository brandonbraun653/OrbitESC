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
            spacing: 14

            ColumnLayout {
                spacing: 3

                Text {
                    text: "LIVE PHASE CURRENTS"
                    color: Theme.textPrimary
                    font.family: Theme.displayFont
                    font.pixelSize: 22
                    font.bold: true
                    font.letterSpacing: 2
                }

                Text {
                    text: "ADC_PHASE_CURRENTS stream, 10-second rolling window"
                    color: Theme.textMuted
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                }
            }

            Item { Layout.fillWidth: true }

            ColumnLayout {
                spacing: 1

                Text { text: "SAMPLE RATE"; color: Theme.textMuted; font.family: Theme.monoFont; font.pixelSize: 9; font.letterSpacing: 1.2 }
                Text {
                    text: app.telemetry.count > 1 ? app.telemetry.sampleRate().toFixed(1) + " Hz" : "—"
                    color: Theme.primary
                    font.family: Theme.monoFont
                    font.pixelSize: 13
                }
            }

            ColumnLayout {
                spacing: 1

                Text { text: "SAMPLES"; color: Theme.textMuted; font.family: Theme.monoFont; font.pixelSize: 9; font.letterSpacing: 1.2 }
                Text {
                    text: app.telemetry.count
                    color: Theme.textPrimary
                    font.family: Theme.monoFont
                    font.pixelSize: 13
                }
            }

            Rectangle {
                Layout.preferredWidth: 1
                Layout.preferredHeight: 34
                color: Theme.border
            }

            RowLayout {
                spacing: 8

                Text {
                    text: "STREAM"
                    color: app.online ? Theme.textPrimary : Theme.textDim
                    font.family: Theme.monoFont
                    font.pixelSize: 10
                    font.letterSpacing: 1.2
                }

                NeonSwitch {
                    id: streamSwitch
                    enabled: app.online
                    onToggled: app.setPhaseCurrentStream(checked)

                    Binding {
                        target: streamSwitch
                        property: "checked"
                        value: app.streamEnabled
                    }
                }
            }

            RowLayout {
                spacing: 8

                Text {
                    text: "AUTO-SCALE"
                    color: Theme.textPrimary
                    font.family: Theme.monoFont
                    font.pixelSize: 10
                    font.letterSpacing: 1.2
                }

                NeonSwitch {
                    id: autoScaleSwitch
                    checked: app.plotAutoScale
                    onToggled: app.plotAutoScale = checked

                    Binding {
                        target: autoScaleSwitch
                        property: "checked"
                        value: app.plotAutoScale
                    }
                }
            }

            NeonButton {
                text: "CLEAR"
                filled: false
                onClicked: app.clearTelemetry()
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: Theme.panel
            border.color: Theme.border
            border.width: 1
            radius: Theme.radius

            PlotCanvas {
                anchors.fill: parent
                anchors.margins: 1
                source: app.telemetry
                autoScale: autoScaleSwitch.checked
                windowSeconds: 10
            }

            Rectangle {
                visible: !app.online
                anchors.centerIn: parent
                width: 300
                height: 76
                radius: Theme.radius
                color: Theme.panelElevated
                border.color: Theme.borderBright
                border.width: 1

                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 4

                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        text: "DEVICE OFFLINE"
                        color: Theme.warning
                        font.family: Theme.displayFont
                        font.pixelSize: 13
                        font.bold: true
                        font.letterSpacing: 1.4
                    }

                    Text {
                        Layout.alignment: Qt.AlignHCenter
                        text: "Connect to enable streaming"
                        color: Theme.textMuted
                        font.family: Theme.monoFont
                        font.pixelSize: 10
                    }
                }
            }
        }
    }
}
