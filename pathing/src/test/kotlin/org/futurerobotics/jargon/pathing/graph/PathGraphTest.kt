package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.pathing.graphPath
import org.futurerobotics.jargon.saveGraph
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import kotlin.random.Random

internal class PathGraphTest {
    @Test
    fun direct() {
        val graph = PathGraph().apply {
            newNode(0.0, 0.0).setName("Start")
                .to(-2.0, 4.0).setName("A")

            newNode(3.0, 3.0).setName("B")
            getNode("A").to("B")

            newNode(3.0, 4.0).setName("End")
                .to(1.0, -2.0)
                .to("B")
        }

        val path = graph.getPath("Start", "End", CurveGenParams(1.5))!!
        XYChart(600, 400).apply {
            styler.apply {
                xAxisMin = -3.0
                xAxisMax = 5.0
                yAxisMin = -3.0
                yAxisMax = 5.0
            }
            graphPath("curve", path, 1000)
        }.saveGraph("builder/PathGraphTest/forward", 300)
    }

    @Test
    fun grid() {
        val size = 10
        val random = Random("Path along a grid".hashCode())
        val graph = PathGraph()
        graph.defaultInterpolator = LinearInterpolator
        repeat(size + 1) { x ->
            repeat(size + 1) { y ->
                val node = graph.newNode(x.toDouble(), y.toDouble()).setName("$x,$y").setTurnAroundWeight(0)
                if (x != 0) node.lineTo(graph.getNode("${x - 1},$y"))
                    .setWeights(random.nextInt(-1000, 1000))
                if (y != 0) node.to(graph.getNode("$x,${y - 1}"))
                    .setWeights(random.nextInt(-1000, 1000))
            }
        }
        val path = graph.getPath("0,0", "$size,$size")!!
        XYChart(500, 400).apply {
            styler.apply {
                xAxisMin = 0.0
                xAxisMax = size.toDouble()
                yAxisMin = 0.0
                yAxisMax = size.toDouble()
            }
            graphPath("curve", path, 1000)
        }.saveGraph("builder/PathGraphTest/grid", 300)
    }
}
