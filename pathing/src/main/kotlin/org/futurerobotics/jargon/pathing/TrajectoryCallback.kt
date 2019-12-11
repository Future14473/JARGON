package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.profile.MotionProfiled
import org.futurerobotics.jargon.running.CompletionCallback
import org.futurerobotics.jargon.util.Stepper
import java.util.*
import kotlin.collections.ArrayList

/**
 * Represents a callback on a [Trajectory]. This can be triggered when a [TrajectoryWithCallbacks] is actually
 * polled.
 */
interface TrajectoryCallback {

    /**
     * The callback to call when run.
     */
    val callback: CompletionCallback

    /**
     * Gets the time of the actual callback, given the [trajectory].
     */
    fun getTime(trajectory: Trajectory): Double
}

/**
 * A [TrajectoryCallback] which triggers when run at the given [distance].
 */
class DistanceCallback(private val distance: Double, override val callback: CompletionCallback) : TrajectoryCallback {

    override fun getTime(trajectory: Trajectory): Double = trajectory.timeAtDistance(distance)
}

/**
 * A [TrajectoryCallback] which triggers when run at the given [time].
 */
class TimeCallback(private val time: Double, override val callback: CompletionCallback) : TrajectoryCallback {

    override fun getTime(trajectory: Trajectory): Double = time
}

/**
 * A callback that is run when the trajectory is complete.
 */
class TrajectoryCompleteCallback(override val callback: CompletionCallback) : TrajectoryCallback {

    override fun getTime(trajectory: Trajectory): Double = trajectory.duration
}

/**
 * A callback that is run when the trajectory is complete.
 */
class TrajectoryStartedCallback

/**
 * A wrapper around a [Trajectory] that also supports it running with [callbacks].
 *
 * @param trajectory the [Trajectory]
 * @param callbacks the callbacks to use.
 */
class TrajectoryWithCallbacks(val trajectory: Trajectory, callbacks: Collection<TrajectoryCallback>) :
    MotionProfiled<MotionState<Pose2d>> {

    private val callbacks =
        callbacks.mapTo(ArrayList()) { it.getTime(trajectory) to it.callback }
            .apply { sortByDescending { it.first } }
            .toCollection(ArrayDeque<Pair<Double, CompletionCallback>>() as Queue<Pair<Double, CompletionCallback>>)

    override val duration: Double
        get() = TODO("not implemented")

    override fun atTime(time: Double): MotionState<Pose2d> = trajectory.atTime(time)

    override fun stepper(): Stepper<MotionState<Pose2d>> = object : Stepper<MotionState<Pose2d>> {
        private val stepper = trajectory.stepper()
        override fun stepTo(step: Double): MotionState<Pose2d> {
            runCallbacks(step)
            return stepper.stepTo(step)
        }
    }

    private fun runCallbacks(time: Double) {
        while (callbacks.isNotEmpty() && callbacks.peek().first <= time) {
            callbacks.remove().second.complete()
        }
    }
}
