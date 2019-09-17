package org.futurerobotics.jargon

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYSeries
import org.knowm.xchart.style.markers.SeriesMarkers
import java.io.File
import java.nio.file.Paths

/**
 * The location to store generated graphs.
 */
const val GRAPH_DIR: String = "./graphs/"

/**
 * Saves this graph at [GRAPH_DIR] location.
 */
fun XYChart.saveGraph(name: String) {
    val file = File(Paths.get(GRAPH_DIR, name).toString())
    file.parentFile.mkdirs()
    BitmapEncoder.saveBitmap(this, file.absolutePath, BitmapEncoder.BitmapFormat.PNG)
}

/**
 * Utilities for creating graphs.
 */
object GraphUtil {
    private val labels = "p0,p1,p2,p3,p4,p5".split(',').toTypedArray()
    /**
     * Creates a graph displaying a spline, with control points.
     */
    fun getSplineGraph(
        steps: Int, vararg pts: Vector2d, toolTip: (Double) -> String? = { null }
    ): XYChart {
        val p0 = pts[0]
        val p1 = pts[1]
        val p2 = pts[2]
        val p3 = pts[3]
        val p4 = pts[4]
        val p5 = pts[5]
        val spline = QuinticSpline.fromControlPoints(p0, p1, p2, p3, p4, p5)
        val xs = mutableListOf<Double>()
        val ys = mutableListOf<Double>()
        val toolTips = mutableListOf<String?>()
        repeat(steps + 1) { i ->
            val t = i.toDouble() / steps
            val v = spline.vec(t)
            xs.add(v.x)
            ys.add(v.y)
            toolTips.add(toolTip(t))
        }
        val chart = XYChart(600, 400)
        // Customize Chart
        chart.title = "Quintic Spline"
        chart.xAxisTitle = "x"
        chart.yAxisTitle = "y"
        val splineSeries = chart.addSeries("Spline", xs, ys)!!
        splineSeries.marker = SeriesMarkers.CROSS
        splineSeries.toolTips = toolTips.toTypedArray()
        val px = doubleArrayOf(p0.x, p1.x, p2.x, p3.x, p4.x, p5.x)
        val py = doubleArrayOf(p0.y, p1.y, p2.y, p3.y, p4.y, p5.y)
        val points = chart.addSeries("control points", px, py)
        points.toolTips = labels
        points.xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Scatter

        chart.styler.isToolTipsEnabled = true
        chart.styler.isToolTipsAlwaysVisible = true
        return chart
    }
}