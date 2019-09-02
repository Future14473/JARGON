package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Vector2d

/** A [Curve] that represents traveling along a Line, starting at [startPos] and ending at endPos */
class Line(private val startPos: Vector2d, endPos: Vector2d) : Curve {

    override val length: Double = startPos distTo endPos
    private val diffNorm = (endPos - startPos).normalized()
    override fun atLength(s: Double): CurvePoint = object :
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
