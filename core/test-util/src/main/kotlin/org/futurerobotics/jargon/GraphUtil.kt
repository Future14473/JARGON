package org.futurerobotics.jargon

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.replaceIf
import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.XYChart
import org.knowm.xchart.style.markers.None
import java.awt.BasicStroke
import java.awt.Color
import java.io.File
import java.nio.file.Paths

/**
 * The default location to store generated graphs.
 */
const val GRAPH_DIR: String = "./graphs/"

/**
 * Saves this graph at [GRAPH_DIR] location.
 */
fun XYChart.saveGraph(name: String, dpi: Int = 72, dir: String = GRAPH_DIR) {
    val file = File(Paths.get(dir, name).toString())
    file.parentFile.mkdirs()
    BitmapEncoder.saveBitmapWithDPI(this, file.absolutePath, BitmapEncoder.BitmapFormat.PNG, dpi)
}

/**
 * Draws a [vec]tor at the given [pos]ition. Optionally [normalize].
 */
fun XYChart.drawVec(
    pos: Vector2d,
    vec: Vector2d,
    normalize: Boolean = true,
    color: Color
) {
    val end = pos + vec.replaceIf(normalize) { it.normalized() }
    val x = doubleArrayOf(pos.x, end.x)
    val y = doubleArrayOf(pos.y, end.y)
    var name: String
    do name = "vec${Math.random()}"
    while (name in seriesMap)

    addSeries(name, x, y).apply {
        marker = None()
        isShowInLegend = false
        lineColor = color
        lineStyle = BasicStroke(1f)
    }
}

/**
 * Draws an arrow at the given [pos]ition, at the given [angle], and the given [magnitude].
 */
fun XYChart.drawArrow(
    pos: Vector2d,
    angle: Double,
    magnitude: Double = 0.5,
    color: Color
) {
    drawVec(pos, Vector2d.polar(magnitude, angle), false, color)
}

