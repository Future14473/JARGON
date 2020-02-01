package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.*
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.Arguments
import org.junit.jupiter.params.provider.MethodSource
import strikt.api.expectThat
import java.util.stream.Stream
import kotlin.random.Random

class QuinticSplineTest {

    @ParameterizedTest
    @MethodSource("splines")
    fun `test point deriv`(
        p1: Vector2d,
        p1Deriv: Vector2d,
        p1SecondDeriv: Vector2d,
        p2: Vector2d,
        p2Deriv: Vector2d,
        p2SecondDeriv: Vector2d,
        spline: VectorFunction
    ) {
        expectThat(p1) isEpsEqTo spline(0.0)

        expectThat(p1Deriv) isEpsEqTo spline.deriv(0.0)

        expectThat(p1SecondDeriv) isEpsEqTo spline.secondDeriv(0.0)

        expectThat(p2) isEpsEqTo spline(1.0)

        expectThat(p2Deriv) isEpsEqTo spline.deriv(1.0)

        expectThat(p2SecondDeriv) isEpsEqTo spline.secondDeriv(1.0)
    }

    companion object {

        @JvmStatic
        fun splines(): Stream<Arguments> {
            val random = Random(seed)
            return Stream.generate {
                val p1 = random.nextVector2d(range)
                val p2 = random.nextVector2d(range)
                val p3 = random.nextVector2d(range)
                val p4 = random.nextVector2d(range)
                val p5 = random.nextVector2d(range)
                val p6 = random.nextVector2d(range)
                val spline = QuinticSpline.fromDerivatives(p1, p2, p3, p4, p5, p6)
                Arguments.of(p1, p2, p3, p4, p5, p6, spline)
            }.limit(100)
        }

        private const val seed = 2134567
        private const val range = 5.0
    }
}
