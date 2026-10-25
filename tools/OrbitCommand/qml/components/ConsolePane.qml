import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand

Item {
    id: root

    property var model
    property bool autoScroll: true

    function severityColor(severity) {
        if (severity === 3)
            return Theme.fault
        if (severity === 2)
            return Theme.warning
        if (severity === 1)
            return Theme.textMuted
        return Theme.textDim
    }

    ListView {
        id: list
        anchors.fill: parent
        model: root.model
        clip: true
        spacing: 2
        ScrollBar.vertical: ScrollBar {}

        onCountChanged: {
            if (root.autoScroll)
                positionViewAtEnd()
        }

        delegate: Rectangle {
            width: list.width
            height: Math.max(24, textItem.implicitHeight + 8)
            color: index % 2 === 0 ? Qt.rgba(1, 1, 1, 0.012) : "transparent"

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 10
                anchors.rightMargin: 10
                spacing: 10

                Text {
                    Layout.preferredWidth: 86
                    text: model.timestamp
                    color: Theme.textDim
                    font.family: Theme.monoFont
                    font.pixelSize: 9
                }

                Text {
                    Layout.preferredWidth: 34
                    text: model.source
                    color: model.source === "ESC" ? Theme.primary : Theme.textMuted
                    font.family: Theme.monoFont
                    font.pixelSize: 9
                    font.bold: true
                    font.letterSpacing: 0.8
                }

                Text {
                    id: textItem
                    Layout.fillWidth: true
                    text: model.text
                    color: root.severityColor(model.severity)
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                    wrapMode: Text.WrapAnywhere
                }
            }
        }
    }
}
