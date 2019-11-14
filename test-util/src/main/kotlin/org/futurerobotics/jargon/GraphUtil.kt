package org.futurerobotics.jargon

import org.knowm.xchart.BitmapEncoder
import org.knowm.xchart.XYChart
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
