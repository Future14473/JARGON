package org.futurerobotics.temporaryname

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.angleNorm
import org.futurerobotics.temporaryname.math.distTo
import kotlin.math.abs

const val ADJUST_THRESH: Int = 10

infix fun Vector2d.errorTo(that: Vector2d): Double {
    val thisLen = this.length
    val otherLen = that.length
    val err = this distTo that
    val totalLen = thisLen + otherLen
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

infix fun Pose2d.errorTo(that: Pose2d): Double =
    kotlin.math.max(vec errorTo that.vec, angleNorm(heading distTo that.heading) / 3)