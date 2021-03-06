package org.futurerobotics.jargon

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.distTo
import kotlin.math.abs

/**
 * The threshold for adjustment in [errorTo] functions
 */
const val ADJUST_THRESH: Double = 10.0

/**
 * Gets a reasonable value for the error from this vector to that vector.
 */
infix fun Vector2d.errorTo(that: Vector2d): Double {
    val thisLen = length
    val otherLen = that.length
    val err = this distTo that
    val totalLen = thisLen + otherLen
    return when {
        totalLen < ADJUST_THRESH -> err
        else -> err / totalLen * ADJUST_THRESH
    }
}

/**
 * Gets a reasonable value for the error from this value to that value.
 */
infix fun Double.errorTo(that: Double): Double {
    val total = abs(this) + abs(that)
    val err = this distTo that
    return when {
        total < ADJUST_THRESH -> err
        else -> err / total * ADJUST_THRESH
    }
}

/**
 * Gets a reasonable value for the error from this pose to that pose.
 */
infix fun Pose2d.errorTo(that: Pose2d): Double =
    kotlin.math.max(vector2d errorTo that.vector2d, angleNorm(heading distTo that.heading) / 3)
