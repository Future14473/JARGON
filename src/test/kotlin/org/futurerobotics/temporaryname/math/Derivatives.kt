@file:JvmName("TestDerivatives")

package org.futurerobotics.temporaryname.math

import kotlin.random.Random

internal fun randomVectorDerivatives(random: Random, range: Double): ValueVectorDerivatives = ValueVectorDerivatives(
    random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
)

internal fun PoseDerivatives.errorTo(that: PoseDerivatives): Double {
    return max(pose errorTo that.pose, poseDeriv errorTo that.poseDeriv, poseSecondDeriv errorTo that.poseSecondDeriv)
}


