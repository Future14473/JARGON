package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.util.Steppable
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents something that has been profiled with a MotionProfile over time,
 * returning [State]s as a representation of motion states.
 */
interface MotionProfiled<out State> : Steppable<Double, State> {
    /** The total duration of this profile, in seconds */
    val duration: Double

    /**
     * Returns the [State] of this motion profiled object at time [time]
     */
    fun atTime(time: Double): State
    /** Gets a [Stepper] for [atTime] */
    override fun stepper(): Stepper<Double, State> =
        Stepper(this::atTime)
}

/**
 * A [MotionProfiled] for one-dimensional motion (state type [MotionState1d])
 */
interface MotionProfile: MotionProfiled<MotionState1d>{
    /**
     * the total distance an object travels on this profile.
     */
    val distance: Double

    /**
     * Returns the [State] of this motion profile after traveling a distance of [distance]
     */
    fun atDistance(distance: Double): MotionState1d
}

