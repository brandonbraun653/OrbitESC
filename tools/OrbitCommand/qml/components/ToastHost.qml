import QtQuick
import QtQuick.Layouts
import OrbitCommand

Item {
    id: root

    function show(severity, message) {
        toastModel.append({
            "severity": severity,
            "message": message
        })
        if (!hideTimer.running)
            hideTimer.start()
    }

    ListModel { id: toastModel }

    Timer {
        id: hideTimer
        interval: 4000
        repeat: true
        onTriggered: {
            if (toastModel.count > 0)
                toastModel.remove(0)
            if (toastModel.count === 0)
                stop()
        }
    }

    ColumnLayout {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        spacing: 8

        Repeater {
            model: toastModel

            Rectangle {
                Layout.fillWidth: true
                implicitHeight: messageText.implicitHeight + 22
                radius: 8
                color: Theme.panelElevated
                border.width: 1
                border.color: {
                    if (model.severity === 2)
                        return Theme.fault
                    if (model.severity === 1)
                        return Theme.warning
                    return Theme.primary
                }

                Rectangle {
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: 3
                    radius: 2
                    color: parent.border.color
                }

                Text {
                    id: messageText
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.leftMargin: 14
                    anchors.rightMargin: 14
                    text: model.message
                    color: Theme.textPrimary
                    font.family: Theme.monoFont
                    font.pixelSize: 11
                    wrapMode: Text.Wrap
                }
            }
        }
    }
}
