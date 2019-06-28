package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Vector2d

/**
 * Holds all useful info about a specific point on a [Curve], which excludes heading.
 *
 * This includes the value, first derivative, and second derivatives, of: position and tangentAngle
 *
 * This is used to avoid re-calculation and possible calculation optimization. All values also have a parallel called
 * directly on `Curve`
 *
 * If you don't care about optimization, see [SimpleCurvePointInfo] which simply delegates to a [Curve]
 * @see PathPointInfo=
 */
interface CurvePointInfo {
    /** The position of this point */
    val position: Vector2d
    /** The [position]'s derivative w/rt arc length at this point */
    val positionDeriv: Vector2d

    /** The [position]'s second derivative w/rt arc length at this point */
    val positionSecondDeriv: Vector2d

    /** The tangent angle of the curve at this point */
    val tanAngle: Double
    /** The [tanAngle]'s derivative w/rt arc length at this point */
    val tanAngleDeriv: Double
    /** The [tanAngle]'s second derivative w/rt arc length at this point. This may be instantaneously discontinuous. */
    val tanAngleSecondDeriv: Double
}

/** returns the curvature at this point, which is exactly equal to [CurvePointInfo.tanAngleDeriv]*/
inline val CurvePointInfo.curvature: Double
    get() = tanAngleDeriv

/** returns the curvature's derivative w/rt arc length at this point, which is exactly equal to [CurvePointInfo.tanAngleSecondDeriv]*/
inline val CurvePointInfo.curvatureDeriv: Double
    get() = tanAngleSecondDeriv

/** A [CurvePointInfo] that simply delegates to a [curve] at the point [s] */
class SimpleCurvePointInfo(
    private val curve: Curve, private val s: Double
) : CurvePointInfo {
    override val position: Vector2d get() = curve.position(s)
    override val positionDeriv: Vector2d get() = curve.positionDeriv(s)
    override val positionSecondDeriv: Vector2d get() = curve.positionSecondDeriv(s)
    override val tanAngle: Double get() = curve.tanAngle(s)
    override val tanAngleDeriv: Double get() = curve.tanAngleDeriv(s)
    override val tanAngleSecondDeriv: Double get() = curve.tanAngleSecondDeriv(s)
}