package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d

/**
 * A [Curve] that is traveling along a straight Line, starting at [startPos] and ending at endPos
 */
class Line(private val startPos: Vector2d, endPos: Vector2d) : Curve {

    override val length: Double = startPos distTo endPos
    private val diffNorm = (endPos - startPos).normalized()
    override fun pointAt(s: Double): CurvePoint = object :
        CurvePoint {
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
 */
class PointTurn(private val point: Vector2d, private val startAngle: Double, endAngle: Double) :
    Path {

    private val diff = endAngle - startAngle
    override val length: Double get() = 1.0
    override val isPointTurn: Boolean = true
    override fun pointAt(s: Double): PathPoint = object :
        PathPoint {
        private val theHeading = startAngle + s * diff
        override val length: Double get() = 1.0
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
