package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.drawArrow
import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.stepToAll
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYSeries
import org.knowm.xchart.style.markers.Circle
import org.knowm.xchart.style.markers.None
import java.awt.Color

fun XYChart.graphPath(name: String, curve: GenericPath<*>, points: Int = 200, color: Color = Color.GREEN): XYSeries {
    val progression = DoubleProgression.fromNumSegments(0.0, curve.length, points)
    return graphPath(name, curve.stepToAll(progression).map { it.position }, color)
}

fun XYChart.graphPath(name: String, points: List<Vector2d>, color: Color = Color.GREEN): XYSeries =
    addSeries(name, points.map { it.x }, points.map { it.y }).apply {
        marker = None()
        xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line
    }.apply {
        lineColor = color
        lineWidth = 1f
    }

fun XYChart.graphPoints(name: String, points: List<Vector2d>): XYSeries =
    addSeries(name, points.map { it.x }, points.map { it.y }).apply {
        marker = Circle()
        xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Scatter
    }

fun XYChart.graphPathWithHeading(
    name: String,
    path: Path,
    points: Int = 200,
    pathColor: Color = Color.GREEN,
    lineSpacing: Int = 20, lineMagnitude: Double = 0.3, lineColor: Color = Color.CYAN
): XYSeries {
    val progression = DoubleProgression.fromNumSegments(0.0, path.length, points)
    val pts = path.stepToAll(progression)

    val graph = graphPath(name, pts.map { it.position }, pathColor)
    pts.forEachIndexed { index, it ->
        if (index % lineSpacing == 0)
            drawPointArrow(it, lineMagnitude, lineColor)
    }
    return graph
}

fun XYChart.drawPointArrow(point: PathPoint, magnitude: Double = 0.5, color: Color = Color.CYAN) {
    drawArrow(point.position, point.heading, magnitude, color)
}
