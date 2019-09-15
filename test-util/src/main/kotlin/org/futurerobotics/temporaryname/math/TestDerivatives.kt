package org.futurerobotics.temporaryname.math

import org.futurerobotics.temporaryname.errorTo
import kotlin.random.Random

/**
 * Creates a random set of [ValueDerivatives] of [Vector2d]
 */
fun randomVectorDerivatives(random: Random, range: Double): ValueDerivatives<Vector2d> =
    ValueDerivatives(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

/**
 * A reasonable value for the error from this pose2d to that pose
 */
fun Derivatives<Pose2d>.errorTo(that: Derivatives<Pose2d>): Double {
    return max(
        value errorTo that.value,
        deriv errorTo that.deriv,
        secondDeriv errorTo that.secondDeriv
    )
}


