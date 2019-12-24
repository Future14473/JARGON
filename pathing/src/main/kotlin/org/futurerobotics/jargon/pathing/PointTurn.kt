package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.asUnmodifiableSet

/**
 * A special "[Path]" that only consists of changing heading from the [startHeading] to by the given [turnAngle],
 * at the given [location].
 *
 * It has a length of 1.0 so that it can fit in to the rest of the trajectory creation system.
 */
class PointTurn(private val location: Vector2d, private val startHeading: Double, private val turnAngle: Double) :
    Path {

    override val length: Double get() = 1.0
    override val stopPoints: Set<Double> = hashSetOf(0.0, 1.0).asUnmodifiableSet()

    override fun pointAt(s: Double): PathPoint = Point(s)

    private inner class Point(s: Double) : PathPoint {
        private val theHeading = startHeading + s * turnAngle
        override val originalLength: Double get() = 1.0
        override val heading: Double get() = theHeading
        override val headingDeriv: Double get() = turnAngle
        override val headingSecondDeriv: Double get() = 0.0
        override val position: Vector2d get() = location
        override val positionDeriv: Vector2d get() = Vector2d.ZERO
        override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
        override val tanAngle: Double get() = theHeading
        override val tanAngleDeriv: Double get() = turnAngle
        override val tanAngleSecondDeriv: Double get() = 0.0
    }
}
