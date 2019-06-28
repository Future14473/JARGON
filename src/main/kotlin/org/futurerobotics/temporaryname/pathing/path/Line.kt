package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Vector2d

/** A Curve that represents traveling along a Line, starting at [startPosition] and ending at [endPos] */
class Line(private val startPosition: Vector2d, endPos: Vector2d) : Curve {
    override val length: Double = startPosition distTo endPos

    private val diffNorm = (endPos - startPosition).normalized()

    override fun position(s: Double): Vector2d = startPosition + diffNorm * s

    override fun positionDeriv(s: Double): Vector2d = diffNorm

    override fun positionSecondDeriv(s: Double): Vector2d = Vector2d.ZERO
    override fun tanAngle(s: Double): Double = diffNorm.angle

    override fun tanAngleDeriv(s: Double): Double = 0.0

    override fun tanAngleSecondDeriv(s: Double): Double = 0.0

    override fun getPointInfo(s: Double): CurvePointInfo = object : CurvePointInfo {
        override val position: Vector2d = position(s)
        override val positionDeriv: Vector2d get() = diffNorm
        override val positionSecondDeriv: Vector2d get() = Vector2d.ZERO
        override val tanAngle: Double get() = diffNorm.angle
        override val tanAngleDeriv: Double get() = 0.0
        override val tanAngleSecondDeriv: Double get() = 0.0
    }

}