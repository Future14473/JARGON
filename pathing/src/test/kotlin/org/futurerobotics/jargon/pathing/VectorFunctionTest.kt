package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.Debug
import org.futurerobotics.jargon.errorTo
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.VectorFunction
import org.futurerobotics.jargon.reportError
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class VectorFunctionTest(private val curve: VectorFunction) {

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
            Debug.breakIf(!errorOk)
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
            Debug.breakIf(!errorOk)
            Assert.assertTrue(errorOk)
        }
    }

    private companion object {

        const val steps = 50_000
        const val epsilon = 1e-6
        const val maxError = 0.001
        const val range = 20.0
        private val random = Random(342567)
        @JvmStatic
        @Parameterized.Parameters
        fun vecs() = List(30) { arrayOf(randomQuinticSpline(random, range)) }
    }
}
