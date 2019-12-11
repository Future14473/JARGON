package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.LinearMotionState
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper

/**
 * Represents a transformed motion of a [MotionProfile], returning some representation of [State] over time.
 */
interface MotionProfiled<out State : Any> : Steppable<State> {

    /** The total duration of this profile, in seconds */
    val duration: Double

    /** Returns the [State] of this motion profiled object at the given [time]. */
    fun atTime(time: Double): State

    /** Gets a [Stepper] that steps along time, returning the [State] at that time. */
    @JvmDefault
    override fun stepper(): Stepper<State> = Stepper(::atTime)
}

/**
 * Represents a Motion Profile: a graph/profile of velocity (and position and acceleration) over time or over a
 * "distance" along _profiled path_.
 *
 * This is also a [MotionProfiled] for one-dimensional motion (state type [LinearMotionState])
 */
interface MotionProfile : MotionProfiled<LinearMotionState> {

    /** The total distance/magnitude of the profiled path. */
    val distance: Double

    /**
     * Returns the [LinearMotionState] of this motion profile after traveling a certain [distance] along
     * the profiled path.
     */
    fun atDistance(distance: Double): LinearMotionState

    /**
     * Gets the time along the profile at a given [distance].
     */
    fun timeAtDistance(distance: Double): Double
}

