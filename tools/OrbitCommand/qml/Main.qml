import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import OrbitCommand
import "components"
import "pages"

ApplicationWindow {
    id: window

    width: 1440
    height: 900
    minimumWidth: 1100
    minimumHeight: 700
    visible: true
    title: "OrbitCommand"
    color: Theme.background
    font.family: Theme.bodyFont
    font.pixelSize: 13

    property int currentPage: app.selectedPage

    onCurrentPageChanged: {
        if (app.selectedPage !== currentPage)
            app.selectedPage = currentPage
    }

    onClosing: function(close) {
        app.saveWindowGeometry(x, y, width, height)
    }

    Component.onCompleted: {
        var geometry = app.windowGeometry()
        if (geometry.valid) {
            width = geometry.width
            height = geometry.height
            x = geometry.x
            y = geometry.y
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        NavRail {
            Layout.fillHeight: true
            Layout.preferredWidth: Theme.railWidth
            currentIndex: window.currentPage
            onPageSelected: function(index) { window.currentPage = index }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            TelemetryHeader {
                Layout.fillWidth: true
                Layout.preferredHeight: Theme.headerHeight
            }

            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: window.currentPage

                DashboardPage {}
                ParametersPage {}
                LiveDataPage {}
                ConsolePage {}
            }
        }
    }

    ToastHost {
        id: toastHost
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 24
        anchors.bottomMargin: 24
        width: 380
    }

    Connections {
        target: app
        function onNotification(severity, message) {
            toastHost.show(severity, message)
        }
    }
}
