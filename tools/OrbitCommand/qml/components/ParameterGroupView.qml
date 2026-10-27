import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand

Item {
    id: root

    property var model
    property string groupTitle: ""

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Text {
                text: root.groupTitle.toUpperCase()
                color: Theme.textPrimary
                font.family: Theme.displayFont
                font.pixelSize: 14
                font.bold: true
                font.letterSpacing: 1.4
            }

            Rectangle {
                Layout.fillWidth: true
                height: 1
                color: Theme.border
            }

            Text {
                text: root.model ? root.model.count() + " PARAMETERS" : ""
                color: Theme.textDim
                font.family: Theme.monoFont
                font.pixelSize: 9
                font.letterSpacing: 1.2
            }
        }

        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            model: root.model
            clip: true
            spacing: 4
            ScrollBar.vertical: ScrollBar {}

            delegate: ParameterField {
                width: list.width
                paramKey: model.key
                paramId: model.paramId
                label: model.label
                paramType: model.type
                value: model.value
                unit: model.unit
                readOnly: model.readOnly
                dirty: model.dirty
                minimum: model.minimum
                maximum: model.maximum
                decimals: model.decimals
                maxLength: model.maxLength
                onValueEdited: function(newValue) { app.setParameterValue(model.paramId, newValue) }
            }
        }
    }
}
