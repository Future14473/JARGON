package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.errorTo
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.reportError
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import strikt.api.expectThat
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class QuinticSplineTest(
    private val p1: Vector2d,
    private val p1Deriv: Vector2d,
    private val p1SecondDeriv: Vector2d,
    private val p2: Vector2d,
    private val p2Deriv: Vector2d,
    private val p2SecondDeriv: Vector2d,
    private val curve: VectorFunction
) {

    @Test
    fun `test point deriv`() {
        expectThat(p1) isEpsEqTo curve(0.0)

        expectThat(p1Deriv) isEpsEqTo curve.deriv(0.0)

        expectThat(p1SecondDeriv) isEpsEqTo curve.secondDeriv(0.0)

        expectThat(p2) isEpsEqTo curve(1.0)

        expectThat(p2Deriv) isEpsEqTo curve.deriv(1.0)

        expectThat(p2SecondDeriv) isEpsEqTo curve.secondDeriv(1.0)
    }

    @Test
    fun `deriv inspect`() {
        testVecDeriv(curve::value, curve::deriv)
    }

    @Test
    fun `secondDeriv inspect`() {
        testVecDeriv(curve::deriv, curve::secondDeriv)
    }

    @Test
    fun `thirdDeriv inspect`() {
        testVecDeriv(curve::secondDeriv, curve::thirdDeriv)
    }

    @Test
    fun `curvatureDeriv inspect`() {
        testDeriv(curve::curvature, curve::curvatureDeriv)
    }

    private inline fun stepT(startOffset: Int = 1, endOffset: Int = 0, block: (Int, Double) -> Unit) {
        for (i in startOffset..(steps - endOffset)) {
            val t = i.toDouble() / steps
            block(i, t)
        }
    }

    private inline fun testDeriv(
        crossinline value: (Double) -> Double, crossinline deriv: (Double) -> Double
    ) {
        reportError {
            stepT(1, 1) { i, t ->
                val aDeriv = (value(t + epsilon) - value(t - epsilon)) / (2 * epsilon)
                val tDeriv = deriv(t)
                val err = aDeriv errorTo tDeriv
                addError(err) { "at $i, actual was $aDeriv, got $tDeriv" }
            }
        }.also {
            println(it.report())
            val errorOk = it.maxError < maxError
            Assert.assertTrue(errorOk)
        }
    }

    private inline fun testVecDeriv(
        crossinline value: (Double) -> Vector2d, crossinline deriv: (Double) -> Vector2d
    ) {
        reportError {
            stepT(1, 1) { i, t ->
                val aDeriv = (value(t + epsilon) - value(t - epsilon)) / (2 * epsilon)
                val tDeriv = deriv(t)
                val err = aDeriv errorTo tDeriv
                addError(err) { "$i, actual was $aDeriv, got $tDeriv" }
            }
        }.also {
            println(it.report())
            val errorOk = it.maxError < maxError
            Assert.assertTrue(errorOk)
        }
    }

    private companion object {

        private const val steps = 50_000
        private const val epsilon = 1e-6
        private const val maxError = 0.001
        private const val seed = 2134567
        private const val range = 10.0
        private val random = Random(seed)

        @JvmStatic
        @Parameterized.Parameters
        fun splines() = List(20) {
            val p1 = random.nextVector2d(range)
            val p2 = random.nextVector2d(range)
            val p3 = random.nextVector2d(range)
            val p4 = random.nextVector2d(range)
            val p5 = random.nextVector2d(range)
            val p6 = random.nextVector2d(range)
            val spline = QuinticSpline.fromDerivatives(p1, p2, p3, p4, p5, p6)
            arrayOf(p1, p2, p3, p4, p5, p6, spline)
        }
    }
}
