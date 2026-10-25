import QtQuick
import QtQuick.Layouts
import OrbitCommand

NeonCard {
    id: root

    property string label: ""
    property string value: "N/A"
    property string unit: ""
    property string detail: ""
    property color valueColor: Theme.textPrimary

    implicitHeight: 116

    Text {
        Layout.fillWidth: true
        text: root.label.toUpperCase()
        color: Theme.textMuted
        font.family: Theme.monoFont
        font.pixelSize: 9
        font.bold: true
        font.letterSpacing: 1.4
        elide: Text.ElideRight
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 6

        Text {
            text: root.value
            color: root.valueColor
            font.family: Theme.displayFont
            font.pixelSize: 26
            font.bold: true
        }

        Text {
            visible: root.unit.length > 0
            Layout.alignment: Qt.AlignBottom
            Layout.bottomMargin: 4
            text: root.unit
            color: Theme.textMuted
            font.family: Theme.monoFont
            font.pixelSize: 10
        }

        Item { Layout.fillWidth: true }
    }

    Text {
        visible: root.detail.length > 0
        Layout.fillWidth: true
        text: root.detail
        color: Theme.textDim
        font.family: Theme.monoFont
        font.pixelSize: 9
        elide: Text.ElideRight
    }
}
