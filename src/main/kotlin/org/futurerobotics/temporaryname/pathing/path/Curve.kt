package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.function.VectorFunction

/**
 * Represents a Parametric curve, ***reparameterized by arc length***, without getHeading.
 *
 * This can be obtained by taking an arbitrary Second-derivative-continuous [VectorFunction] and reparameterizing using the classes given
 * in the reparam package.
 *
 * _Due to being parameterized by arc length, there are several nice properties/relationships among the values given
 * by this interface useful for both implementation and usage. Read the docs!!_
 *
 * All derivatives are with respect to arc length.
 * @see PathSegment
 */
interface Curve {
    /**
     * The total (arc) length of this curve/path, and the maximum `s` value the functions
     * of this interface can take and return something that makes sense.
     */
    val length: Double

    /**
     *  The position [s] units along this path
     *  @see positionDeriv
     */
    fun position(s: Double): Vector2d

    /**
     *  The [position]'s derivative with w/rt arc length, [s] units along this path.
     *
     *  Since the path is parameterized by arc length,
     *  This will always have getHeading length of 1, and can be derived from [tanAngle].
     *  @see positionSecondDeriv
     *  @see tanAngle
     */
    fun positionDeriv(s: Double): Vector2d

    /**
     *  The [position]'s second derivative w/rt arc length, [s] units along this path.
     *
     *  Since the path is parameterized by arc length,
     *  This will always be perpendicular to [positionDeriv], and have a length of [tanAngleDeriv].
     *
     *  This is equal to (tanAngleDeriv) zcross (positionDeriv)
     *
     *  Consider using [tanAngleDeriv] instead to save 10 nanoseconds of computing time.
     */
    fun positionSecondDeriv(s: Double): Vector2d

    /**
     * The angle of the tangent vector at the point [s] units along this path.
     *
     * Since this path is reparameterized by arc length,
     * This is enough information to derive [positionDeriv], as it always has a getHeading length of 1.
     *
     * This is equal to [positionDeriv].angle
     */
    fun tanAngle(s: Double): Double

    /**
     * The tanAngleDeriv at [s] units along this path.
     *
     * Since the path is reparameterized by arc length, this is exactly equal to the [curvature] of the path.
     *  (`omega = c*|v| = c`).
     *  Also, this has the same magnitude as of [positionSecondDeriv].
     * [positionSecondDeriv] always faces perpendicularly left of [positionDeriv] if tanAngleDeriv is positive, and right otherwise.
     *
     * This is equal to [positionDeriv] cross [positionSecondDeriv]
     * @see positionSecondDeriv
     */
    fun tanAngleDeriv(s: Double): Double


    /**
     * The [tanAngleDeriv]'v derivative w/rt arc length [s] units along this path.
     * This is equal to the [curvatureDeriv], for easier calculation.
     *
     * This may be instantaneously discontinuous.
     */
    fun tanAngleSecondDeriv(s: Double): Double

    /**
     * Returns a [CurvePointInfo] containing info about the point [s] units along this path.
     */
    fun getPointInfo(s: Double): CurvePointInfo = SimpleCurvePointInfo(this, s)

    /**
     *  Returns a List of [CurvePointInfo] for all the points [allS]'s units along the path. Here is room for some optimizations.
     * @see getPointInfo
     */
    fun getAllPointInfo(allS: List<Double>): List<CurvePointInfo> = allS.map(::getPointInfo)

}

/**
 * The derivative of curvature [s] units along this path.
 *
 * This is exactly equal to [Curve.tanAngleSecondDeriv], and is here for convenience only
 */
inline fun Curve.curvatureDeriv(s: Double): Double = tanAngleSecondDeriv(s)

/**
 * The curvature of this path [s] units along this path.
 *
 * This is exactly equal to [Curve.tanAngleDeriv], and is here for convenience only
 */
inline fun Curve.curvature(s: Double): Double = tanAngleDeriv(s)