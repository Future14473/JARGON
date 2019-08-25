@file:Suppress("NOTHING_TO_INLINE")

package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.function.VectorFunction
import org.futurerobotics.temporaryname.util.Steppable
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents a curve or path, with the given [Point]. This generic intermediary is to solve the generics problem
 * so that a [Path] is-a [Curve] while implementing interfaces with different generics.
 */
interface GenericPath<out Point : CurvePoint> : Steppable<Double, Point> {
    /**
     * The total (arc) length of this curve/path, and the maximum `s` value the functions
     * of this interface can take and return something that makes sense.
     */
    val length: Double

    /**
     * Returns a Point containing info about the point [s] units along this path.
     */
    fun atLength(s: Double): Point

    /**
     * Gets a stepper that steps through points along the path, returning a Point containing info about that point.
     */
    override fun stepper(): Stepper<Double, Point> =
        Stepper { atLength(it) }
}

/**
 * Represents a parametric curve. ***parameterized by arc length***, without heading. This is essentially a [Path] but
 * without heading.
 *
 * All data about points along the curve is contained within a [CurvePoint] via [pointAt][GenericPath.atLength] (position, derivatives, etc)
 *
 * This can be obtained by taking an arbitrary Second-derivative-continuous [VectorFunction] and reparameterizing it.
 *
 * All derivatives are with respect to arc length.
 *
 *
 * This is actually a typealias for [GenericPath]<[CurvePoint]> to workaround solve some generics issues with interfaces,
 * but should work as you expect (a [Path] is-a [Curve])
 *
 * @see CurvePoint
 * @see Path
 */
typealias Curve = GenericPath<CurvePoint>

/**
 * Represents a path for a robot to follow, along a curve ***parameterized by arc length***, _including_ heading info.
 *
 * All data about points along the curve is contained within a [PathPoint] via [atLength] (position, derivatives, etc)
 *
 * This can be obtained by attaching a [HeadingProvider] to an arbitrary [Curve] via [ComponentPath] or equivalently via
 * [addHeading]
 *
 * Note that a [Path] is-a [Curve], which I scratched my head a lot to make work while other things work too
 * @see PathPoint
 * @see Curve
 */
interface Path : GenericPath<PathPoint> {

    /**
     * @return true if this is a point turn; i.e. position does not change but heading does.
     * If so, length will have to be 1.0, and all position-related info should return 0 or Vector.ZERO
     */
    val isPointTurn: Boolean get() = false
}

//fun test() {
//    val path: Path? = null
//    val curve: Curve? = path //path is-A Curve!!! YES
//}
