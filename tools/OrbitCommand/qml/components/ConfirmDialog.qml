import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand

Dialog {
    id: root

    property string message: ""
    property string confirmText: "CONFIRM"

    signal confirmed()

    modal: true
    width: 420
    padding: 0
    closePolicy: Popup.CloseOnEscape

    background: Rectangle {
        color: Theme.panelElevated
        radius: Theme.radius
        border.color: Theme.borderBright
        border.width: 1
    }

    contentItem: ColumnLayout {
        spacing: 18

        Text {
            Layout.fillWidth: true
            text: "CONFIRM ACTION"
            color: Theme.warning
            font.family: Theme.displayFont
            font.pixelSize: 14
            font.bold: true
            font.letterSpacing: 1.6
        }

        Text {
            Layout.fillWidth: true
            text: root.message
            color: Theme.textPrimary
            font.family: Theme.monoFont
            font.pixelSize: 12
            wrapMode: Text.Wrap
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Item { Layout.fillWidth: true }

            NeonButton {
                text: "CANCEL"
                filled: false
                onClicked: root.close()
            }

            NeonButton {
                text: root.confirmText
                danger: true
                onClicked: {
                    root.close()
                    root.confirmed()
                }
            }
        }
    }
}
