package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.PointPath
import org.futurerobotics.jargon.pathing.graphPathWithHeading
import org.futurerobotics.jargon.pathing.multiplePath
import org.futurerobotics.jargon.saveGraph
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart
import strikt.api.expectThat
import strikt.assertions.isA
import kotlin.random.Random

internal class PathGraphTest {
    @Test
    fun direct() {
        val graph = PathGraph().apply {
            addNode(0.0, 0.0).name("Start")
                .splineTo(-2.0, 4.0).name("A")

            addNode(3.0, 3.0).name("B")
            getNode("A").splineTo("B")

            addNode(3.0, 4.0).name("End")
                .splineTo(1.0, -2.0)
                .splineTo("B")
        }

        val paths = graph.getPaths("Start", "End", curveGenParams = CurveGenParams(1.5))!!

        val path = multiplePath(paths)
        XYChart(600, 400).apply {
            styler.apply {
                xAxisMin = -3.0
                xAxisMax = 5.0
                yAxisMin = -3.0
                yAxisMax = 5.0
            }
            graphPathWithHeading("path", path, 1000)
        }.saveGraph("builder/PathGraphTest/direct", 300)
    }

    @Test
    fun grid() {
        val size = 10
        val random = Random("Path along a grid".hashCode())
        val graph = PathGraph()
        graph.defaultInterpolator = LinearInterpolator
        repeat(size + 1) { x ->
            repeat(size + 1) { y ->
                val node = graph.addNode(x.toDouble(), y.toDouble()).name("$x,$y").setTurnAroundWeight(0)
                if (x != 0) node.splineTo(graph.getNode("${x - 1},$y"))
                    .setWeights(random.nextInt(-1000, 1000))
                if (y != 0) node.splineTo(graph.getNode("$x,${y - 1}"))
                    .setWeights(random.nextInt(-1000, 1000))
            }
        }
        val path = graph.getPath("0,0", "$size,$size", curveGenParams = CurveGenParams(1.3))
        XYChart(500, 400).apply {
            styler.apply {
                xAxisMin = 0.0
                xAxisMax = size.toDouble()
                yAxisMin = 0.0
                yAxisMax = size.toDouble()
            }
            graphPathWithHeading("path", path, 1000, lineSpacing = 5)
        }.saveGraph("builder/PathGraphTest/grid", 300)
    }

    @Test
    fun singlePoint() {
        val graph = PathGraph()
        graph.addNode(Vector2d.ZERO).name("Hey")
        val path = graph.getPath("Hey", "Hey")
        expectThat(path).isA<PointPath>()
    }
}
