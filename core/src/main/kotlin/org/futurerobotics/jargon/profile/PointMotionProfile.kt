package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.LinearMotionState

/**
 * A profile which consists of a single [state] at time 0, and has a total length and duration of 0.0.
 */
class PointMotionProfile(private val state: LinearMotionState) : ForwardMotionProfile {

    @JvmOverloads
    constructor(position: Double = 0.0, deriv: Double = 0.0, secondDeriv: Double = 0.0)
            : this(LinearMotionState(position, deriv, secondDeriv))

    override val duration: Double get() = 0.0

    override fun atTime(time: Double): LinearMotionState = if (time == 0.0) state else state.afterTime(time)

    override val length: Double get() = 0.0

    override fun atLength(length: Double): LinearMotionState = if (length == 0.0) state else state.atDistance(length)

    override fun timeAtLength(length: Double): Double = if (length == 0.0) 0.0 else state.timeElapsedAtDistance(length)
}
