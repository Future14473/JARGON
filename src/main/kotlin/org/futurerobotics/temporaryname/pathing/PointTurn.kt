package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Vector2d

/**
 * A special "[Path]" that only consists of changing heading, not position.
 */
class PointTurn(private val point: Vector2d, private val startAngle: Double, endAngle: Double) : Path {
    private val diff = endAngle - startAngle
    override val length: Double get() = 1.0
    override val isPointTurn: Boolean = true

    override fun pointAt(s: Double): PathPoint = object : PathPoint {
        private val theHeading = startAngle + s * diff
        override val length: Double get() = this@PointTurn.length
        override val heading: Double get() = theHeading
        override val headingDeriv: Double get() = diff
        override val headingSecondDeriv: Double get() = 0.0
        override val position: Vector2d get() = point
        override val positionDeriv: Vector2d get() = Vector2d.ZERO
        override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
        override val tanAngle: Double get() = heading
        override val tanAngleDeriv: Double get() = theHeading
        override val tanAngleSecondDeriv: Double get() = 0.0
    }
}
