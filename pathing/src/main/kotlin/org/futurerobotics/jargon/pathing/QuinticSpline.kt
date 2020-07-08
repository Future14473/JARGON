package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*

/** Creators for a Quintic Spline, defined by two quintic polynomials for the x and y components. */
object QuinticSpline {

    /**
     * Creates a quintic spline using the control points of a Bezier spline.
     */
    @JvmStatic
    fun fromControlPoints(
        p0: Vector2d, p1: Vector2d, p2: Vector2d, p3: Vector2d, p4: Vector2d, p5: Vector2d
    ) = ComponentVectorFunction(
        QuinticPolynomial.fromControlPoints(
            p0.x, p1.x, p2.x, p3.x, p4.x, p5.x
        ), QuinticPolynomial.fromControlPoints(
            p0.y, p1.y, p2.y, p3.y, p4.y, p5.y
        )
    )

    /**
     * Creates a quintic spline given the value, and first and second derivatives of each of the endpoints.
     */
    @JvmStatic
    fun fromDerivatives(
        start: Vector2d,
        startDeriv: Vector2d,
        startSecondDeriv: Vector2d,
        end: Vector2d,
        endDeriv: Vector2d,
        endSecondDeriv: Vector2d
    ): ComponentVectorFunction = ComponentVectorFunction(
        QuinticPolynomial.fromDerivatives(
            start.x, startDeriv.x, startSecondDeriv.x, end.x, endDeriv.x, endSecondDeriv.x
        ), QuinticPolynomial.fromDerivatives(
        start.y, startDeriv.y, startSecondDeriv.y, end.y, endDeriv.y, endSecondDeriv.y
    )
    )

    /**
     * Creates a quintic spline given the value, and first and second derivatives of each of the
     * end points.
     */
    @JvmStatic
    fun fromDerivatives(
        start: MotionState<Vector2d>, end: MotionState<Vector2d>
    ): ComponentVectorFunction = fromDerivatives(
        start.value, start.deriv, start.secondDeriv, end.value, end.deriv, end.secondDeriv
    )
}

/**
 * A Quintic Polynomial function, specified by coefficients,
 * in the form:
 *
 * [a]t^5+[b]t^4+[c]t^3+[d]t^2+[e]t+[f]
 */
@Suppress("KDocMissingDocumentation")
class QuinticPolynomial(
    @JvmField val a: Double,
    @JvmField val b: Double,
    @JvmField val c: Double,
    @JvmField val d: Double,
    @JvmField val e: Double,
    @JvmField val f: Double
) : RealFunction {

    constructor(vec: DoubleArray) : this(vec[0], vec[1], vec[2], vec[3], vec[4], vec[5])

    override fun value(t: Double): Double = ((((a * t + b) * t + c) * t + d) * t + e) * t + f

    override fun deriv(t: Double): Double = (((5 * a * t + 4 * b) * t + 3 * c) * t + 2 * d) * t + e

    override fun secondDeriv(t: Double): Double = ((20 * a * t + 12 * b) * t + 6 * c) * t + 2 * d

    override fun thirdDeriv(t: Double): Double = (60 * a * t + 24 * b) * t + 6 * c

    override fun toString(): String = "QuinticPoly(%.4ft^5+%.4ft^4+%.4ft^3+%.4ft^2+%.4ft+%.4f)".format(a, b, c, d, e, f)

    companion object {
        private val fromControlPoints = matOf(
            -1, 5, -10, 10, -5, 1 to
                    5, -20, 30, -20, 5, 0 to
                    -10, 30, -30, 10, 0, 0 to
                    10, -20, 10, 0, 0, 0 to
                    -5, 5, 0, 0, 0, 0 to
                    1, 0, 0, 0, 0, 0
        )

        @Suppress("UnnecessaryVariable")
        internal fun fromControlPoints(
            p0: Double, p1: Double, p2: Double, p3: Double, p4: Double, p5: Double
        ): QuinticPolynomial {
            val vec = doubleArrayOf(p0, p1, p2, p3, p4, p5)
            return QuinticPolynomial(fromControlPoints * vec)
        }

        private val fromDerivatives = matOf(
            -6, -3, -0.5, 6, -3, 0.5 to
                    15, 8, 1.5, -15, 7, -1 to
                    -10, -6, -1.5, 10, -4, 0.5 to
                    0, 0, 0.5, 0, 0, 0 to
                    0, 1, 0, 0, 0, 0 to
                    1, 0, 0, 0, 0, 0
        )

        /** Creates a Quintic polynomial defined by the value and first and second derivatives of endpoints. */
        @Suppress("UnnecessaryVariable")
        @JvmStatic
        fun fromDerivatives(
            start: Double,
            startDeriv: Double,
            startSecondDeriv: Double,
            end: Double,
            endDeriv: Double,
            endSecondDeriv: Double
        ): QuinticPolynomial {
            val vec = doubleArrayOf(start, startDeriv, startSecondDeriv, end, endDeriv, endSecondDeriv)
            return QuinticPolynomial(fromDerivatives * vec)
        }

        /**
         * Creates a Quintic polynomial defined by start and endpoint [MotionState]s.
         */
        @JvmStatic
        fun fromDerivatives(
            start: AnyMotionState<Double>, end: AnyMotionState<Double>
        ): QuinticPolynomial =
            fromDerivatives(
                start.value, start.deriv, start.secondDeriv, end.value, end.deriv, end.secondDeriv
            )
    }
}

