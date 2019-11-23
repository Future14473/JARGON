package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.errorTo
import kotlin.random.Random

/**
 * Creates a random set of [ValueMotionState] of [Vector2d]
 */
fun randomVectorDerivatives(random: Random, range: Double): ValueMotionState<Vector2d> =
    ValueMotionState(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

/**
 * A reasonable value for the error from this pose2d to that pose
 */
fun MotionState<Pose2d>.errorTo(that: MotionState<Pose2d>): Double {
    return max(
        value errorTo that.value,
        deriv errorTo that.deriv,
        secondDeriv errorTo that.secondDeriv
    )
}


