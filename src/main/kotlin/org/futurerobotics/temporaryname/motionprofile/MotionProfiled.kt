package org.futurerobotics.temporaryname.motionprofile

import org.futurerobotics.temporaryname.util.Stepper

/**
 * Represents something that has been profiled with a MotionProfile over time,
 * returning [State]s as a representation of motion states.
 */
interface MotionProfiled<out State> {
    /** The total duration of this profile, in seconds */
    val duration: Double
    /** The total length of this profile, in distance. */
    val length: Double

    /**
     * Gets the state given by this profile at the duration [time] after the start of this profile.
     *
     * If the supplied value [time] is (far) outside the range [0..[duration]], the return value is undefined.
     */
    fun atTime(time: Double): State

    /**
     * Gets the state given by this profile after traveling a certain [distance], NOT DISPLACEMENT along this profile.
     *
     * If the supplied value [distance] is (far) outside the range [0..[length]], the return value is undefined.
     */
    fun atDistance(distance: Double): State

    /** Gets a [Stepper] for [atTime] */
    fun timeStepper(): Stepper<Double, State> = Stepper(this::atTime)

    /** Gets a [Stepper] for [atDistance] */
    fun distanceStepper(): Stepper<Double, State> = Stepper(this::atDistance)
}

/**
 * A [MotionProfiled] for one-dimensional motion (state type [MotionState1d])
 */
typealias MotionProfile = MotionProfiled<MotionState1d>

