import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand

Rectangle {
    id: root

    property string paramKey: ""
    property int paramId: -1
    property string label: ""
    property int paramType: 0
    property var value: 0
    property string unit: ""
    property bool readOnly: false
    property bool dirty: false
    property double minimum: 0
    property double maximum: 100
    property int decimals: 0
    property int maxLength: 32

    signal valueEdited(var newValue)

    objectName: paramKey.length > 0 ? "oc.parameters.field." + paramKey : ""
    Accessible.name: label

    implicitHeight: 52
    color: dirty ? Qt.rgba(1, 0.17, 0.84, 0.045) : "transparent"
    border.color: dirty ? Qt.rgba(1, 0.17, 0.84, 0.35) : Theme.border
    border.width: 1
    radius: 6

    readonly property string displayValue: {
        if (paramType === 1)
            return value ? "TRUE" : "FALSE"
        if (paramType === 8 || paramType === 7)
            return value === undefined || value === null ? "" : String(value)
        var numberValue = Number(value)
        if (isNaN(numberValue))
            return "0"
        return numberValue.toFixed(decimals)
    }

    function formattedValue() {
        return displayValue
    }

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 14
        anchors.rightMargin: 14
        spacing: 12

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 1

            Text {
                Layout.fillWidth: true
                text: root.label
                color: root.readOnly ? Theme.textMuted : Theme.textPrimary
                font.family: Theme.monoFont
                font.pixelSize: 12
                elide: Text.ElideRight
            }

            Text {
                visible: root.readOnly
                text: "READ ONLY"
                color: Theme.textDim
                font.family: Theme.monoFont
                font.pixelSize: 8
                font.letterSpacing: 1.2
            }
        }

        Text {
            visible: root.readOnly
            Layout.preferredWidth: 220
            horizontalAlignment: Text.AlignRight
            text: root.displayValue
            color: Theme.textMuted
            font.family: Theme.monoFont
            font.pixelSize: 12
            elide: Text.ElideRight
        }

        Loader {
            visible: !root.readOnly
            Layout.preferredWidth: 220
            Layout.preferredHeight: 32

            sourceComponent: {
                if (root.paramType === 1)
                    return boolEditor
                if (root.paramType === 8)
                    return stringEditor
                if (root.paramType === 5 || root.paramType === 6)
                    return doubleEditor
                return integerEditor
            }
        }

        Text {
            visible: root.unit.length > 0
            Layout.preferredWidth: 44
            text: root.unit
            color: Theme.textMuted
            font.family: Theme.monoFont
            font.pixelSize: 10
            elide: Text.ElideRight
        }

        Rectangle {
            visible: root.dirty
            width: 8
            height: 8
            radius: 4
            color: Theme.secondary
        }
    }

    Component {
        id: boolEditor

        NeonSwitch {
            id: boolSwitch
            objectName: root.paramKey.length > 0 ? "oc.parameters.editor." + root.paramKey : ""
            Accessible.name: root.label
            anchors.right: parent.right
            anchors.verticalCenter: parent.verticalCenter
            onToggled: root.valueEdited(checked)

            Binding {
                target: boolSwitch
                property: "checked"
                value: root.value === true
            }
        }
    }

    Component {
        id: stringEditor

        TextField {
            id: stringField
            objectName: root.paramKey.length > 0 ? "oc.parameters.editor." + root.paramKey : ""
            Accessible.name: root.label
            maximumLength: root.maxLength
            selectByMouse: true
            color: Theme.textPrimary
            font.family: Theme.monoFont
            font.pixelSize: 12
            onEditingFinished: root.valueEdited(text)

            background: Rectangle {
                radius: 5
                color: Theme.background
                border.color: stringField.activeFocus ? Theme.primary : Theme.border
                border.width: 1
            }

            Binding {
                target: stringField
                property: "text"
                value: root.value === undefined || root.value === null ? "" : String(root.value)
            }
        }
    }

    Component {
        id: doubleEditor

        TextField {
            id: doubleField
            objectName: root.paramKey.length > 0 ? "oc.parameters.editor." + root.paramKey : ""
            Accessible.name: root.label
            horizontalAlignment: Text.AlignRight
            selectByMouse: true
            color: Theme.textPrimary
            font.family: Theme.monoFont
            font.pixelSize: 12
            validator: DoubleValidator {
                bottom: root.minimum
                top: root.maximum
                decimals: root.decimals
                notation: DoubleValidator.StandardNotation
            }

            onEditingFinished: {
                var parsed = Number(text)
                if (!isNaN(parsed))
                    root.valueEdited(parsed)
            }

            background: Rectangle {
                radius: 5
                color: Theme.background
                border.color: doubleField.activeFocus ? Theme.primary : Theme.border
                border.width: 1
            }

            Binding {
                target: doubleField
                property: "text"
                value: Number(root.value).toFixed(root.decimals)
            }
        }
    }

    Component {
        id: integerEditor

        SpinBox {
            id: intBox
            objectName: root.paramKey.length > 0 ? "oc.parameters.editor." + root.paramKey : ""
            Accessible.name: root.label
            from: Math.max(root.minimum, -2147483647)
            to: Math.min(root.maximum, 2147483647)
            editable: true
            onValueModified: root.valueEdited(value)

            contentItem: TextInput {
                text: intBox.textFromValue(intBox.value, intBox.locale)
                color: Theme.textPrimary
                font.family: Theme.monoFont
                font.pixelSize: 12
                horizontalAlignment: Qt.AlignRight
                verticalAlignment: Qt.AlignVCenter
                selectByMouse: true
                readOnly: !intBox.editable
                validator: intBox.validator
                inputMethodHints: Qt.ImhFormattedNumbersOnly
            }

            background: Rectangle {
                radius: 5
                color: Theme.background
                border.color: intBox.activeFocus ? Theme.primary : Theme.border
                border.width: 1
            }

            Binding {
                target: intBox
                property: "value"
                value: Math.round(Number(root.value))
            }
        }
    }
}
