package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.TAU
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.pathing.Line
import org.futurerobotics.jargon.pathing.graphPath
import org.futurerobotics.jargon.pathing.graphPoints
import org.futurerobotics.jargon.saveGraph
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChart

internal class CurveBuilderTest {
    @Test
    fun `curve builder graph`() {
        val points = listOf(
            Vector2d(0.0, 0.0),
            Vector2d(1.0, 2.0),
            Vector2d(4.0, 2.0),
            Vector2d(2.0, -2.0),
            Vector2d(4.0, 0.0),
            Vector2d(4.5, 3.0),
            Vector2d(0.0, 4.0),
            Vector2d(-2.0, -2.0),
            Vector2d(-3.0, 3.0),
            Vector2d(0.0, 5.0),
            Vector2d(3.0, 3.0),
            Vector2d(5.0, -3.0)
        )

        val curve = CurveBuilder(CurveGenParams(derivMagnitudeMultiplier = 1.3))
            .toPoints(points.map { Waypoint((it)) })
            .addCurve(Line(Vector2d(5.0, -3.0), Vector2d(0.0, -2.0)))
            .build()

        XYChart(600, 400).apply {
            styler.apply {
                xAxisMin = -3.0
                xAxisMax = 5.0
                yAxisMin = -3.0
                yAxisMax = 5.0
            }
            graphPath("curve", curve, 1000)
            graphPoints("points", points)
        }.saveGraph("curves/CurveBuilderTest", 300)
    }

    @Test
    fun `path builder graph`() {

        val builder = PathBuilder()
            .setDefaultHeadingProvider(TurnOffsetToContinuation(0.0))
            .toPoint(0.0, 0.0, 90 * deg).initialHeading(90 * deg)
            .toPoint(1.0, 2.0)
            .toPoint(4.0, 2.0).heading(TurnByContinuation(90 * deg))
            .toPoint(2.0, -2.0).heading(TurnOffsetToContinuation(0.0))
            .toPoint(4.0, 0.0).heading(TurnToContinuation(TAU))
            .toPoint(4.5, 3.0)
            .toPoint(0.0, 4.0)
            .toPoint(-2.0, -2.0)
            .toPoint(-3.0, 3.0)
            .toPoint(0.0, 5.0)
            .toPoint(3.0, 3.0)
            .toPoint(5.0, -3.0)
        val pts = builder.getPoints()
        val curve = builder.build()
        XYChart(600, 400).apply {
            styler.apply {
                xAxisMin = -3.0
                xAxisMax = 5.0
                yAxisMin = -3.0
                yAxisMax = 5.0
            }
            graphPath("path", curve, 1000)
            graphPoints("points", pts)
        }.saveGraph("curves/PathBuilderTest", 300)
    }
}
