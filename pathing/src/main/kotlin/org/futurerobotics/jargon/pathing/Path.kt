@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.VectorFunction
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Common superclass of [Curve] and [Path], outputting a given [Point] type.
 *
 * This allows for some unification between [Curve] and [Path].
 *
 * AnyPath<*> is the superclass of both [Curve] and [Path].
 */
interface AnyPath<out Point : CurvePoint> : Steppable<Point> {

    /**
     * The total (arc) length of this curve/path.
     */
    val length: Double

    /**
     * Returns a Point containing info about the point [s] units along this path (in arc length).
     *
     * If value given is outside 0 to [length], the values returned may not be meaningful.
     */
    fun pointAt(s: Double): Point

    /** Gets a stepper that steps through points along the path. */
    override fun stepper(): Stepper<Point> = Stepper(::pointAt)

    //TODO: revise this
    /** A set of points that it is required for the bot to stop at. */
    val stopPoints: Set<Double>
        get() = emptySet()

    /**
     * Gets the point at the start of this curve/path.
     */
    fun startPoint(): Point = pointAt(0.0)

    /**
     * Gets the point at the end of this curve/path.
     */
    fun endPoint(): Point = pointAt(length)
}

/**
 * Represents a parametric curve. ***parameterized by arc length***, without heading. This is essentially a [Path] but
 * without heading.
 *
 * All data about points along the curve is contained within a [CurvePoint] via [pointAt][AnyPath.pointAt]
 * (position, derivatives, etc)
 *
 * This can be obtained by taking an arbitrary Second-derivative-continuous [VectorFunction] and reparameterizing it.
 *
 * All derivatives are with respect to arc length.
 *
 * @see CurvePoint
 * @see Path
 */
interface Curve : AnyPath<CurvePoint>

/**
 * Represents a path for a robot to follow, along a curve ***parameterized by arc length***, _including_ heading info.
 *
 * All data about points along the curve is contained within a [PathPoint] via [pointAt] (position, derivatives, etc)
 *
 * This can be obtained by attaching a [HeadingProvider] to an arbitrary [Curve] via [CurveHeadingPath] or equivalently via
 * [addHeading]
 *
 * @see PathPoint
 * @see Curve
 */
interface Path : AnyPath<PathPoint>

/**
 * Returns this [AnyPath] as a [Curve].
 */
fun AnyPath<*>.asCurve(): Curve = when (this) {
    is Curve -> this
    is CurveHeadingPath -> curve
    else -> object : Curve, AnyPath<CurvePoint> by this {}
}
