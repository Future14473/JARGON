@file:JvmName("TestDerivatives")

package org.futurerobotics.temporaryname.math

import org.futurerobotics.temporaryname.errorTo
import kotlin.random.Random

fun randomVectorDerivatives(random: Random, range: Double): ValueDerivatives<Vector2d> =
    ValueDerivatives<Vector2d>(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

fun Derivatives<Pose2d>.errorTo(that: Derivatives<Pose2d>): Double {
    return max(
        value errorTo that.value,
        deriv errorTo that.deriv,
        secondDeriv errorTo that.secondDeriv
    )
}


