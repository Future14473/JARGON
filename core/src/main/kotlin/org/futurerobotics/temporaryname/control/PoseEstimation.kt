package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.cosc
import org.futurerobotics.temporaryname.math.sinc
import org.futurerobotics.temporaryname.mechanics.Motion

/**
 * Non-linearly tracks the global pose, given relative pose velocities.
 */
class GlobalPoseTracker(initialPose: Pose2d = Pose2d.ZERO) : BaseObserver<Motion<Pose2d>, Any, Pose2d>() {

    init {
        state = initialPose
    }

    override fun getState(measurement: Motion<Pose2d>, lastSignal: Any, elapsedSeconds: Double): Pose2d {
        val (v, dTheta) = measurement.v * elapsedSeconds
        val (x, y) = v
        val sinc = sinc(dTheta)
        val cosc = cosc(dTheta)
        val dPose = Pose2d(
            Vector2d(sinc * x - cosc * y, cosc * x + sinc * y).rotated(state.heading), dTheta
        )
        return (state + dPose).normalizeAngle()
    }

    override fun start() {
    }

    override fun stop() {
    }
}