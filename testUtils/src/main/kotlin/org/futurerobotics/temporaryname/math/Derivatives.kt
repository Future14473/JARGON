@file:JvmName("TestDerivatives")

package org.futurerobotics.temporaryname.math

import org.futurerobotics.temporaryname.errorTo
import kotlin.random.Random

fun randomVectorDerivatives(random: Random, range: Double): ValueVectorDerivatives =
    ValueVectorDerivatives(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

fun PoseDerivatives.errorTo(that: PoseDerivatives): Double {
    return max(
        pose errorTo that.pose,
        poseDeriv errorTo that.poseDeriv,
        poseSecondDeriv errorTo that.poseSecondDeriv
    )
}


