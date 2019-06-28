package org.futurerobotics.temporaryname.math

import kotlin.math.abs

const val ADJUST_THRESH = 10

infix fun Vector2d.errorTo(that: Vector2d): Double {
    val thislen = this.length
    val otherlen = that.length
    val err = this distTo that
    val totalLen = thislen + otherlen
    return when {
        totalLen < ADJUST_THRESH -> err
        else -> err / totalLen * ADJUST_THRESH
    }
}

infix fun Double.errorTo(that: Double): Double {
    val total = abs(this) + abs(that)
    val err = this distTo that
    return when {
        total < ADJUST_THRESH -> err
        else -> err / total * ADJUST_THRESH
    }
}

infix fun Pose2d.errorTo(that: Pose2d) =
    kotlin.math.max(vec errorTo that.vec, angleNorm(heading distTo that.heading) / 3)