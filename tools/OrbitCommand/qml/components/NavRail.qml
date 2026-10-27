import QtQuick
import QtQuick.Layouts
import OrbitCommand

Rectangle {
    id: root

    property int currentIndex: 0
    signal pageSelected(int index)

    color: Theme.panel
    border.color: Theme.border
    border.width: 1

    readonly property var pages: [
        { key: "dashboard", label: "DASH", glyph: "◈" },
        { key: "parameters", label: "PARAM", glyph: "⚙" },
        { key: "liveData", label: "LIVE", glyph: "∿" },
        { key: "console", label: "LOG", glyph: "▤" }
    ]

    ColumnLayout {
        anchors.fill: parent
        anchors.topMargin: 18
        anchors.bottomMargin: 18
        spacing: 6

        Rectangle {
            Layout.alignment: Qt.AlignHCenter
            width: 42
            height: 42
            radius: 21
            color: Qt.rgba(0, 0.9, 1, 0.08)
            border.color: Theme.primary
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: "OC"
                color: Theme.primary
                font.family: Theme.displayFont
                font.pixelSize: 12
                font.bold: true
                font.letterSpacing: 1
            }
        }

        Item { Layout.preferredHeight: 18 }

        Repeater {
            model: root.pages

            Rectangle {
                id: item
                objectName: "oc.nav." + modelData.key
                Accessible.name: modelData.label
                Accessible.role: Accessible.Button
                Layout.fillWidth: true
                Layout.preferredHeight: 62
                color: root.currentIndex === index ? Theme.panelElevated : "transparent"

                Rectangle {
                    visible: root.currentIndex === index
                    anchors.left: parent.left
                    anchors.verticalCenter: parent.verticalCenter
                    width: 3
                    height: 34
                    radius: 2
                    color: Theme.primary
                }

                Column {
                    anchors.centerIn: parent
                    spacing: 4

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: modelData.glyph
                        color: root.currentIndex === index ? Theme.primary : Theme.textMuted
                        font.family: Theme.bodyFont
                        font.pixelSize: 18
                    }

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: modelData.label
                        color: root.currentIndex === index ? Theme.textPrimary : Theme.textDim
                        font.family: Theme.monoFont
                        font.pixelSize: 9
                        font.bold: true
                        font.letterSpacing: 1.2
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.pageSelected(index)
                }
            }
        }

        Item { Layout.fillHeight: true }

        Text {
            Layout.alignment: Qt.AlignHCenter
            text: "v0.1"
            color: Theme.textDim
            font.family: Theme.monoFont
            font.pixelSize: 9
        }
    }
}
