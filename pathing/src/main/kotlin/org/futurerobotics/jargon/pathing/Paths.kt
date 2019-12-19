package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.asUnmodifiableSet

/**
 * A [Curve] that is traveling along a straight Line, starting at [startPos] and ending at `endPos.`
 */
class Line(private val startPos: Vector2d, endPos: Vector2d) : Curve {

    override val length: Double = startPos distTo endPos
    private val diffNorm = (endPos - startPos).normalized()
    override fun pointAt(s: Double): CurvePoint = Point(s)

    private inner class Point(s: Double) : CurvePoint {
        override val length: Double get() = this@Line.length
        override val position: Vector2d = startPos + diffNorm * s //not get
        override val positionDeriv: Vector2d get() = diffNorm
        override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
        override val tanAngle: Double get() = diffNorm.angle
        override val tanAngleDeriv: Double get() = 0.0
        override val tanAngleSecondDeriv: Double get() = 0.0
    }
}

/**
 * A special "[Path]" that only consists of changing heading, not position.
 * It has a `length` of 1.0 so that it can fit in to the rest of the trajectory creation system.
 */
class PointTurn(private val point: Vector2d, private val startHeading: Double, private val turnAngle: Double) : Path {

    override val length: Double get() = 1.0
    override val stopPoints: Set<Double> = hashSetOf(0.0, 1.0).asUnmodifiableSet()

    override fun pointAt(s: Double): PathPoint = Point(s)

    private inner class Point(s: Double) : PathPoint {
        private val theHeading = startHeading + s * turnAngle
        override val length: Double get() = 1.0
        override val heading: Double get() = theHeading
        override val headingDeriv: Double get() = turnAngle
        override val headingSecondDeriv: Double get() = 0.0
        override val position: Vector2d get() = point
        override val positionDeriv: Vector2d get() = Vector2d.ZERO
        override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
        override val tanAngle: Double get() = theHeading
        override val tanAngleDeriv: Double get() = turnAngle
        override val tanAngleSecondDeriv: Double get() = 0.0
    }
}
