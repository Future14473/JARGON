package org.futurerobotics.jargon.pathing.reparam

import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.pathing.QuinticSpline
import org.futurerobotics.jargon.reportError
import org.futurerobotics.jargon.saveGraph
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import org.junit.runners.Parameterized.Parameters
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYSeries
import org.knowm.xchart.style.markers.SeriesMarkers
import kotlin.math.PI
import kotlin.math.pow
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class ReparamCurveTest(private val func: VectorFunction, private val curve: ReparamCurve) {

    @Test
    fun `report samples`() {
        println("Num samples: ${(curve.mapping as SamplesReparamMapping).numSamples}")
    }

    @Test
    fun `reparameterization inspect`() {
        testValue({ it }, { curve.mapping.tOfS(it) }, 0.001, 0.0)
    }

    @Test
    fun `position inspect`() {
        testVector({ func.value(it) }, { curve.pointAt(it).position }, 0.001, 0.0)
    }

    @Test
    fun `positionDeriv inspect`() {
        testVector({ func.deriv(it).normalized() }, { curve.pointAt(it).positionDeriv }, 0.002, 0.001)
    }

    @Test
    fun `positionSecondDeriv inspect`() {
        testVector({
                       val deriv = func.deriv(it)
                       val secondDeriv = func.secondDeriv(it)
                       val z = (secondDeriv cross deriv) / deriv.lengthSquared.pow(2)
                       Vector2d(deriv.y * z, -deriv.x * z)
                   }, { curve.pointAt(it).positionSecondDeriv }, 0.005, 0.001)
    }

    @Test
    fun `tanAngle inspect`() {
        testAngle({ func.deriv(it).angle }, { curve.pointAt(it).tanAngle }, 0.001, 0.0)
    }

    @Test
    fun `tanAngleDeriv inspect`() {
        testValue({ func.curvature(it) }, { curve.pointAt(it).tanAngleDeriv }, 0.002, 0.001)
    }

    @Test
    fun `tanAngleSecondDeriv inspect`() {
        testValue(
            { func.curvatureDeriv(it) / func.deriv(it).length },
            { curve.pointAt(it).tanAngleSecondDeriv },
            0.05,
            0.001
        )
    }

    private inline fun testVector(
        crossinline trueValT: (Double) -> Vector2d,
        crossinline testValS: (Double) -> Vector2d,
        maxError: Double,
        offset: Double
    ) {
        testStep(trueValT, testValS, { this distTo it }, offset, maxError)
    }

    private inline fun testValue(
        crossinline trueValT: (Double) -> Double,
        crossinline testValS: (Double) -> Double,
        maxError: Double,
        offset: Double
    ) {
        testStep(trueValT, testValS, { this distTo it }, offset, maxError)
    }

    private inline fun testAngle(
        crossinline trueValT: (Double) -> Double,
        crossinline testValS: (Double) -> Double,
        maxError: Double,
        offset: Double
    ) {
        testStep(trueValT, testValS, { angleNorm(distTo(it)) }, offset, maxError)
    }

    private inline fun <T> testStep(
        crossinline trueValT: (Double) -> T,
        crossinline testValS: (Double) -> T,
        crossinline getError: T.(T) -> Double,
        offset: Double,
        maxError: Double
    ) {
        val offsetInt = (steps * offset).toInt()

        reportError {
            var s = 0.0
            for (i in 0..(steps - offsetInt)) {
                val t = i.toDouble() / steps
                if (i >= offsetInt) {
                    val trueVal = trueValT(t)
                    val testVal = testValS(s)
                    val err = trueVal.getError(testVal)
                    addError(err) { "$t, true is $trueVal, got $testVal" }
                }
                s += func.deriv((i + 0.5) / steps).length / steps
            }
        }.let {
            println(Thread.currentThread().stackTrace[1].methodName + ":")
            println(it.report())
            println()
            assertTrue(it.averageError <= maxError)
        }
    }

    companion object {
        const val steps = 20_000
        private const val seed = 34226
        private const val minDiff = 2.0
        private const val maxDiff = 8.0
        private val random = Random(seed)
        @JvmStatic
        @Parameters
        fun curves(): List<Array<Any>> {
            val list = mutableListOf<Array<Any>>()

            repeat(30) {
                val p0 = random.nextVector2d(
                    minDiff
                )
                val diff = random.nextDouble(
                    minDiff,
                    maxDiff
                )
                val angle = random.nextDouble() * TAU
                val p5 = p0 + Vector2d.polar(diff, angle)
                val p0Deriv =
                    Vector2d.polar(diff * random.nextDouble(0.2, 1.0), angle + random.nextDouble(-PI / 3, PI / 3))
                val p0SecondDeriv = Vector2d.ZERO
                val p5Deriv =
                    Vector2d.polar(diff * random.nextDouble(0.2, 1.0), angle + random.nextDouble(-PI / 3, PI / 3))
                val p5SecondDeriv = Vector2d.ZERO
                val p1 = p0 + p0Deriv / 5
                val p2 = p0SecondDeriv / 20 + 2 * p1 - p0
                val p4 = p5 - p5Deriv / 5
                val p3 = p5SecondDeriv / 20 + 2 * p4 - p5

                val spline = QuinticSpline.fromControlPoints(p0, p1, p2, p3, p4, p5)
                val xs = mutableListOf<Double>()
                val ys = mutableListOf<Double>()
                val toolTips = mutableListOf<String?>()
                repeat(30 + 1) { i ->
                    val t = i.toDouble() / 30
                    val v = spline(t)
                    xs.add(v.x)
                    ys.add(v.y)
                    toolTips.add(null)
                }
                val chart = XYChart(600, 400)
                chart.title = "Quintic Spline"
                chart.xAxisTitle = "x"
                chart.yAxisTitle = "y"
                val splineSeries = chart.addSeries("Spline", xs, ys)!!
                splineSeries.marker = SeriesMarkers.CROSS
                splineSeries.toolTips = toolTips.toTypedArray()
                val px = doubleArrayOf(p0.x, p1.x, p2.x, p3.x, p4.x, p5.x)
                val py = doubleArrayOf(p0.y, p1.y, p2.y, p3.y, p4.y, p5.y)
                val points = chart.addSeries("control points", px, py)
                points.toolTips = "p0,p1,p2,p3,p4,p5".split(',').toTypedArray()
                points.xySeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Scatter
                chart.styler.isToolTipsEnabled = true
                chart.styler.isToolTipsAlwaysVisible = true
                chart.saveGraph("RandomSpline/$it")
                val func = QuinticSpline.fromControlPoints(p0, p1, p2, p3, p4, p5)

                list.add(arrayOf(func, func.reparameterizeToCurve()))
            }
            return list
        }
    }
}

