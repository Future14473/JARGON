package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.errorTo
import org.futurerobotics.jargon.math.function.QuinticSpline
import kotlin.random.Random

/**
 * Creates a random set of [MotionState] of [Vector2d]
 */
fun randomVectorDerivatives(random: Random, range: Double): MotionState<Vector2d> =
    MotionState(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

/**
 * Creates a random [QuinticSpline].
 */
fun randomQuinticSpline(random: Random, range: Double): QuinticSpline = QuinticSpline.fromDerivatives(
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range)
)

/**
 * A reasonable value for the error from this pose2d to that pose
 */
fun MotionState<Pose2d>.errorTo(that: MotionState<Pose2d>): Double {
    return maxOf(
        value errorTo that.value,
        deriv errorTo that.deriv,
        secondDeriv errorTo that.secondDeriv
    )
}


