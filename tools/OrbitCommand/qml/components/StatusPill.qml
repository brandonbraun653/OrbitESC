import QtQuick
import OrbitCommand

Rectangle {
    id: root

    property string text: ""
    property string tone: "idle"

    Accessible.name: text
    Accessible.role: Accessible.StaticText

    readonly property color toneColor: {
        if (tone === "ok")
            return Theme.success
        if (tone === "warn")
            return Theme.warning
        if (tone === "error")
            return Theme.fault
        return Theme.textMuted
    }

    implicitWidth: label.implicitWidth + 24
    implicitHeight: 24
    radius: 12
    color: Qt.rgba(toneColor.r, toneColor.g, toneColor.b, 0.1)
    border.color: Qt.rgba(toneColor.r, toneColor.g, toneColor.b, 0.55)
    border.width: 1

    Text {
        id: label
        anchors.centerIn: parent
        text: root.text.toUpperCase()
        color: root.toneColor
        font.family: Theme.monoFont
        font.pixelSize: 10
        font.bold: true
        font.letterSpacing: 1.2
    }
}
