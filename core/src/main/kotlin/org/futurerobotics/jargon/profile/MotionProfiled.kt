package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.mechanics.LinearMotionState
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents a transformed motion of a [MotionProfile], returning some representation of [State] over time.
 */
interface MotionProfiled<out State : Any> : Steppable<Double, State> {

    /** The total duration of this profile, in seconds */
    val duration: Double

    /**
     * Returns the [State] of this motion profiled object at time [time]
     */
    fun atTime(time: Double): State

    /** Gets a [Stepper] for [atTime] */
    override fun stepper(): Stepper<Double, State> = Stepper(this::atTime)
}

/**
 * Represents a Motion Profile: a graph/profile of velocity (and position and acceleration) over time or distance.
 *
 * This is also a [MotionProfiled] for one-dimensional motion (state type [LinearMotionState])
 */
interface MotionProfile : MotionProfiled<LinearMotionState> {

    /**
     * the total distance an object travels on this profile.
     */
    val distance: Double

    /**
     * Returns the [LinearMotionState] of this motion profile after traveling a distance of [distance]
     */
    fun atDistance(distance: Double): LinearMotionState
}

