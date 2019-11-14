package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.stepToAll
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYSeries
import org.knowm.xchart.style.markers.Circle
import org.knowm.xchart.style.markers.None

fun XYChart.graphPath(name: String, curve: GenericPath<*>, points: Int = 200): XYSeries {
    val progression = DoubleProgression.fromNumSegments(0.0, curve.length, points)
    return graphPath(name, curve.stepToAll(progression).map { it.position })
}

fun XYChart.graphPath(name: String, points: List<Vector2d>): XYSeries =
    addSeries(name, points.map { it.x }, points.map { it.y }).apply {
        marker = None()
        xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line
    }

fun XYChart.graphPoints(name: String, points: List<Vector2d>): XYSeries =
    addSeries(name, points.map { it.x }, points.map { it.y }).apply {
        marker = Circle()
        xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Scatter
    }
