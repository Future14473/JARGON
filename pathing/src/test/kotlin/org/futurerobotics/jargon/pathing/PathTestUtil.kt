package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.ComponentVectorFunction
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.nextVector2d
import kotlin.random.Random

private fun CurvePoint.getContentArray() = arrayOf(
    position,
    positionDeriv,
    positionSecondDeriv,
    tanAngle,
    tanAngleDeriv,
    tanAngleSecondDeriv
)

private fun PathPoint.getContentArray() = arrayOf(
    position,
    positionDeriv,
    positionSecondDeriv,
    tanAngle,
    tanAngleDeriv,
    tanAngleSecondDeriv,
    heading,
    headingDeriv,
    headingSecondDeriv
)

infix fun PathPoint.contentEquals(other: PathPoint): Boolean = getContentArray() contentEquals other.getContentArray()

infix fun CurvePoint.contentEquals(other: CurvePoint): Boolean = getContentArray() contentEquals other.getContentArray()

fun randomVectorDerivatives(random: Random, range: Double): MotionState<Vector2d> =
    MotionState(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

fun randomQuinticSpline(random: Random, range: Double): ComponentVectorFunction = QuinticSpline.fromDerivatives(
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range)
)
