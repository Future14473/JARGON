package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.RealMotionState
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents something that has [State] profiled over time.
 *
 * @see MotionProfile
 */
interface TimeProfiled<out State> {

    /** The total duration of the profile, in seconds. */
    val duration: Double

    /** Returns the [State] of this motion profiled object at the given [time]. */
    fun atTime(time: Double): State

    /** Gets a [Stepper] that steps along time, returning the [State] at that time (like [atTime]) */
    fun timeStepper(): Stepper<State> = Stepper(::atTime)
}

/**
 * Represents something that has [State] profiled over time as well as over distance (see [TimeProfiled]).
 * In order for this to be true, this profile must be moving always forward or always backward.
 *
 * This allows for polling of state over distance as well as time.
 *
 * @see ForwardMotionProfile
 */
interface TimeDistanceProfiled<out State> : TimeProfiled<State> {

    /** The total distance of the profile. */
    val distance: Double

    /** Returns the [RealMotionState] of this motion profile at the given [distance]. */
    fun atDistance(distance: Double): State

    /** Gets the time of the profile at a given [distance]. */
    fun timeAtDistance(distance: Double): Double

    //distance stepper?
}
