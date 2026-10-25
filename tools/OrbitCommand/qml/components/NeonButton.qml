import QtQuick
import QtQuick.Controls
import OrbitCommand

Button {
    id: control

    property bool danger: false
    property bool filled: true
    property color accent: danger ? Theme.fault : Theme.primary

    implicitHeight: 36
    implicitWidth: Math.max(88, contentItem.implicitWidth + leftPadding + rightPadding)
    leftPadding: 16
    rightPadding: 16

    contentItem: Text {
        text: control.text
        color: control.enabled ? (control.danger ? Theme.fault : Theme.primary) : Theme.textDim
        font.family: Theme.monoFont
        font.pixelSize: 11
        font.bold: true
        font.letterSpacing: 1.2
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        radius: Theme.radius
        color: {
            if (!control.enabled)
                return Theme.panel
            if (control.down)
                return control.danger ? Qt.rgba(1, 0.09, 0.27, 0.28) : Qt.rgba(0, 0.9, 1, 0.22)
            if (control.hovered)
                return control.danger ? Qt.rgba(1, 0.09, 0.27, 0.18) : Qt.rgba(0, 0.9, 1, 0.14)
            return control.filled ? (control.danger ? Qt.rgba(1, 0.09, 0.27, 0.1) : Qt.rgba(0, 0.9, 1, 0.07)) : "transparent"
        }
        border.color: control.enabled ? control.accent : Theme.border
        border.width: 1

        Rectangle {
            anchors.fill: parent
            anchors.margins: -3
            radius: parent.radius + 3
            color: "transparent"
            border.color: Qt.rgba(control.accent.r, control.accent.g, control.accent.b, control.enabled ? 0.12 : 0.03)
            border.width: 1
        }
    }
}
