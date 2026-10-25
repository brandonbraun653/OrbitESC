import QtQuick
import OrbitCommand

Canvas {
    id: canvas

    property var source: null
    property bool autoScale: true
    property double windowSeconds: 10

    onSourceChanged: requestPaint()

    Timer {
        interval: 33
        running: true
        repeat: true
        onTriggered: canvas.requestPaint()
    }

    onPaint: {
        var ctx = getContext("2d")
        var mono = "10px '" + Theme.monoFont + "'"
        var smallMono = "9px '" + Theme.monoFont + "'"
        var left = 58
        var right = 18
        var top = 16
        var bottom = 30
        var plotWidth = Math.max(1, width - left - right)
        var plotHeight = Math.max(1, height - top - bottom)

        ctx.clearRect(0, 0, width, height)
        ctx.fillStyle = Theme.panel
        ctx.fillRect(0, 0, width, height)

        var data = source ? source.windowData(windowSeconds) : null
        var count = data ? data.count : 0
        var minValue = -5
        var maxValue = 5

        if (autoScale && data && data.max > data.min) {
            var padding = Math.max(0.1, (data.max - data.min) * 0.12)
            minValue = data.min - padding
            maxValue = data.max + padding
        } else if (!autoScale) {
            minValue = -5
            maxValue = 5
        }

        ctx.strokeStyle = Theme.border
        ctx.lineWidth = 1
        ctx.fillStyle = Theme.textDim
        ctx.font = smallMono
        for (var i = 0; i <= 5; ++i) {
            var gy = top + plotHeight * i / 5
            ctx.beginPath()
            ctx.moveTo(left, gy)
            ctx.lineTo(width - right, gy)
            ctx.stroke()

            var gridValue = maxValue - (maxValue - minValue) * i / 5
            ctx.fillText(gridValue.toFixed(1), 6, gy + 3)
        }

        for (var gx = 0; gx <= 5; ++gx) {
            var x = left + plotWidth * gx / 5
            ctx.beginPath()
            ctx.moveTo(x, top)
            ctx.lineTo(x, top + plotHeight)
            ctx.stroke()

            var seconds = -windowSeconds + windowSeconds * gx / 5
            ctx.fillText(seconds.toFixed(1) + "s", x - 16, height - 9)
        }

        ctx.save()
        ctx.beginPath()
        ctx.rect(left, top, plotWidth, plotHeight)
        ctx.clip()

        function xFor(t) {
            return left + (1.0 + t / windowSeconds) * plotWidth
        }

        function yFor(v) {
            if (maxValue === minValue)
                return top + plotHeight / 2
            return top + plotHeight * (maxValue - v) / (maxValue - minValue)
        }

        function drawSeries(values, color) {
            if (!values || values.length < 2)
                return

            ctx.beginPath()
            for (var index = 0; index < values.length; ++index) {
                var px = xFor(data.times[index])
                var py = yFor(values[index])
                if (index === 0)
                    ctx.moveTo(px, py)
                else
                    ctx.lineTo(px, py)
            }

            ctx.globalAlpha = 0.18
            ctx.lineWidth = 7
            ctx.strokeStyle = color
            ctx.stroke()

            ctx.globalAlpha = 1.0
            ctx.lineWidth = 1.8
            ctx.strokeStyle = color
            ctx.stroke()
        }

        if (count > 0) {
            drawSeries(data.ia, Theme.primary)
            drawSeries(data.ib, Theme.secondary)
            drawSeries(data.ic, Theme.success)
        }

        ctx.restore()

        ctx.font = mono
        ctx.fillStyle = Theme.primary
        ctx.fillText("Ia", width - right - 72, top + 12)
        ctx.fillStyle = Theme.secondary
        ctx.fillText("Ib", width - right - 48, top + 12)
        ctx.fillStyle = Theme.success
        ctx.fillText("Ic", width - right - 24, top + 12)

        if (count === 0) {
            ctx.fillStyle = Theme.textDim
            ctx.font = "12px '" + Theme.monoFont + "'"
            ctx.fillText("WAITING FOR PHASE-CURRENT SAMPLES", left + 20, top + plotHeight / 2)
        }
    }
}
