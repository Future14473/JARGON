package org.futurerobotics.jargon.math.function

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.nextVector2d
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class QuinticSplineTest(
    private val p1: Vector2d,
    private val p1Deriv: Vector2d,
    private val p1SecondDeriv: Vector2d,
    private val p2: Vector2d,
    private val p2Deriv: Vector2d,
    private val p2SecondDeriv: Vector2d,
    private val spline: QuinticSpline
) {

    @Test
    fun `test point deriv`() {
        println(p1)
        Assert.assertTrue(p1 epsEq spline.vec(0.0))

        println(p1Deriv)
        Assert.assertTrue(p1Deriv epsEq spline.vecDeriv(0.0))

        println(p1SecondDeriv)
        Assert.assertTrue(p1SecondDeriv epsEq spline.vecSecondDeriv(0.0))

        println(p2)
        Assert.assertTrue(p2 epsEq spline.vec(1.0))

        println(p2Deriv)
        Assert.assertTrue(p2Deriv epsEq spline.vecDeriv(1.0))

        println(p2SecondDeriv)
        Assert.assertTrue(p2SecondDeriv epsEq spline.vecSecondDeriv(1.0))
    }

    companion object {
        private const val seed = 2134567
        private const val range = 5.0
        @Parameterized.Parameters
        @JvmStatic
        fun params(): Collection<Array<Any>> {
            val random = Random(seed)
            return List(200) {
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
}
