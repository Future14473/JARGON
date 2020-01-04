package org.futurerobotics.jargon.math.function

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Vector2d

/** Represents a Quintic Spline, defined by two quintic polynomials for the x and y components. */
class QuinticSpline(x: QuinticPolynomial, y: QuinticPolynomial) : ComponentVectorFunction(x, y) {

    override fun toString(): String = "QuinticSpline(x: $x, y: $y)"

    companion object {
        /**
         * Creates a quintic spline using the control points of a Bezier spline.
         */
        @JvmStatic
        fun fromControlPoints(
            p0: Vector2d, p1: Vector2d, p2: Vector2d, p3: Vector2d, p4: Vector2d, p5: Vector2d
        ): QuinticSpline = QuinticSpline(
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
        ): QuinticSpline = QuinticSpline(
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
        fun fromDerivatives(
            start: MotionState<Vector2d>, end: MotionState<Vector2d>
        ): QuinticSpline = fromDerivatives(
            start.value, start.deriv, start.secondDeriv, end.value, end.deriv, end.secondDeriv
        )
    }
}
