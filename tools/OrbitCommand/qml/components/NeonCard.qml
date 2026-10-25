import QtQuick
import QtQuick.Layouts
import OrbitCommand

Rectangle {
    id: root

    default property alias content: body.data
    property string title: ""
    property color accent: Theme.primary

    radius: Theme.radius
    color: Theme.panel
    border.color: Theme.border
    border.width: 1
    implicitWidth: 240
    implicitHeight: 120

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 10

        Text {
            visible: root.title.length > 0
            Layout.fillWidth: true
            text: root.title.toUpperCase()
            color: Theme.textMuted
            font.family: Theme.monoFont
            font.pixelSize: 10
            font.bold: true
            font.letterSpacing: 1.6
            elide: Text.ElideRight
        }

        ColumnLayout {
            id: body
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 8
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 1
        height: 1
        radius: parent.radius
        color: Qt.rgba(root.accent.r, root.accent.g, root.accent.b, 0.35)
    }
}
