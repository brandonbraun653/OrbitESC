pragma Singleton
import QtQuick

QtObject {
    readonly property color background: "#070A12"
    readonly property color panel: "#0E1420"
    readonly property color panelElevated: "#131C2A"
    readonly property color panelHover: "#172235"
    readonly property color border: "#1C2A3A"
    readonly property color borderBright: "#2B4258"
    readonly property color primary: "#00E5FF"
    readonly property color primaryDim: "#007A8A"
    readonly property color secondary: "#FF2BD6"
    readonly property color success: "#39FF14"
    readonly property color warning: "#FFB300"
    readonly property color fault: "#FF1744"
    readonly property color textPrimary: "#E6F1FF"
    readonly property color textMuted: "#7C8DA6"
    readonly property color textDim: "#4A5B72"

    readonly property string displayFont: "Orbitron"
    readonly property string bodyFont: "JetBrains Mono"
    readonly property string monoFont: "JetBrains Mono"

    readonly property int radius: 10
    readonly property int spacing: 12
    readonly property int pageMargin: 24
    readonly property int headerHeight: 82
    readonly property int railWidth: 92
}
