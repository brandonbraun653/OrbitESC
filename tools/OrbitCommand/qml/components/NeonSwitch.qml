import QtQuick
import QtQuick.Controls
import OrbitCommand

Switch {
    id: control

    implicitWidth: 42
    implicitHeight: 22
    spacing: 0

    contentItem: Item {}

    indicator: Rectangle {
        anchors.fill: parent
        radius: height / 2
        color: control.checked ? Qt.rgba(0, 0.9, 1, 0.16) : Theme.background
        border.color: control.enabled ? (control.checked ? Theme.primary : Theme.border) : Theme.border
        border.width: 1

        Rectangle {
            width: 16
            height: 16
            radius: 8
            y: 2
            x: control.checked ? parent.width - width - 2 : 2
            color: control.enabled ? (control.checked ? Theme.primary : Theme.textMuted) : Theme.textDim
            Behavior on x {
                NumberAnimation { duration: 120; easing.type: Easing.OutCubic }
            }
        }
    }
}
