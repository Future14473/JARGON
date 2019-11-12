@file:Suppress("NOTHING_TO_INLINE")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.function.VectorFunction
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper
import java.io.Serializable

/**
 * Represents a curve or path, with the given [Point]. This generic intermediary is to solve the generics problem
 * so that you are can have a type that is either a [Path] or a [Curve]
 *
 * GenericCurve<? extends CurvePoint> or GenericCurve<out CurvePoint> is the superclass of both [Curve] and [Path]
 */
interface GenericPath<out Point : CurvePoint> : Steppable<Double, Point>, Serializable {

    /**
     * The total (arc) length of this curve/path, and the maximum `s` value the functions
     * of this interface can take and return something that makes sense.
     */
    val length: Double

    /** Returns a Point containing info about the point [s] units along this path. */
    fun pointAt(s: Double): Point

    /** Gets a stepper that steps through points along the path, returning a Point containing info about that point. */
    @JvmDefault
    override fun stepper(): Stepper<Double, Point> = Stepper { pointAt(it) }

    /** A sorted list of points that it is required for the bot to stop on this path.*/
    @JvmDefault
    val stopPoints: List<Double>
        get() = emptyList()
}

/**
 * Represents a parametric curve. ***parameterized by arc length***, without heading. This is essentially a [Path] but
 * without heading.
 *
 * All data about points along the curve is contained within a [CurvePoint] via [pointAt][GenericPath.pointAt]
 * (position, derivatives, etc)
 *
 * This can be obtained by taking an arbitrary Second-derivative-continuous [VectorFunction] and reparameterizing it.
 *
 * All derivatives are with respect to arc length.
 *
 * @see CurvePoint
 * @see Path
 */
interface Curve : GenericPath<CurvePoint>

/**
 * Represents a path for a robot to follow, along a curve ***parameterized by arc length***, _including_ heading info.
 *
 * All data about points along the curve is contained within a [PathPoint] via [pointAt] (position, derivatives, etc)
 *
 * This can be obtained by attaching a [HeadingProvider] to an arbitrary [Curve] via [ComponentPath] or equivalently via
 * [addHeading]
 *
 * @see PathPoint
 * @see Curve
 */
interface Path : GenericPath<PathPoint>

/**
 * Returns this path as a [Curve] instead.
 */
fun GenericPath<*>.asCurve(): Curve = when (this) {
    is Curve -> this
    is ComponentPath -> curve
    else -> object : Curve, GenericPath<CurvePoint> by this {}
}
