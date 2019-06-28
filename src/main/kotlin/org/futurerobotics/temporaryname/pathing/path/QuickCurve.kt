package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.function.QuinticPolynomial
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.pathing.path.reparam.ReparamCurve
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration

/**
 * Utilities for easy creation of [Curve]s, from [QuinticSpline]s and [Line]s.
 */
object QuickCurve {
    /**
     * Creates a [QuinticSpline] [ReparamCurve] from the control points of a Bezier spline, then turns it into a
     * curve via [reparamByIntegration]
     */
    @JvmStatic
    fun splineFromControlPoints(
        p0: Vector2d, p1: Vector2d, p2: Vector2d, p3: Vector2d, p4: Vector2d, p5: Vector2d
    ): ReparamCurve = QuinticSpline.fromControlPoints(p0, p1, p2, p3, p4, p5).reparamByIntegration()

    /**
     * Creates a [QuinticSpline] [ReparamCurve] the value, and first and second derivatives of each of the endpoints.
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
}