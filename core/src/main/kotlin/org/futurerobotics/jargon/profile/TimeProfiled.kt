package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.LinearMotionState
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents something that has [State] profiled over time.
 */
interface TimeProfiled<out State : Any> : Steppable<State> {

    /** The total duration of the profile, in seconds. */
    val duration: Double

    /** Returns the [State] of this motion profiled object at the given [time]. */
    fun atTime(time: Double): State

    /** Gets a [Stepper] that steps along time, returning the [State] at that time (akin to [atTime]) */
    override fun stepper(): Stepper<State> = Stepper(::atTime)
}

/**
 * Represents something that has [State] profiled over distance, as well as time (see [TimeProfiled]).
 *
 * This allows for polling of state over distance as well as time.
 */
interface TimeDistanceProfiled<out State : Any> : TimeProfiled<State> {

    /** The total distance of the profile. */
    val distance: Double

    /** Returns the [LinearMotionState] of this motion profile at the given [distance]. */
    fun atDistance(distance: Double): State

    /** Gets the time of the profile at a given [distance]. */
    fun timeAtDistance(distance: Double): Double
}