package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.cosc
import org.futurerobotics.jargon.math.sinc
import org.futurerobotics.jargon.mechanics.GlobalToBot
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState3

/**
 * Non-linearly tracks the _global_ pose, given _bot_ pose velocities.
 */
class GlobalPoseObserver(initialPose: Pose2d = Pose2d.ZERO) : BaseObserver<MotionOnly<Pose2d>, Any, Pose2d>() {

    init {
        state = initialPose
    }

    override fun getState(measurement: MotionOnly<Pose2d>, lastSignal: Any, elapsedSeconds: Double): Pose2d {
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


/**
 * A wrapper around another controller that first maps reference from global to bot first.
 *
 * The current state supplied will always be [Pose2d.ZERO], and reference moves around.
 */
class GlobalToBotMotionController(
    private val baseController: Controller<MotionState3<Pose2d>, Pose2d, MotionOnly<Pose2d>>
) : Controller<MotionState3<Pose2d>, Pose2d, MotionOnly<Pose2d>> {

    override val signal: MotionOnly<Pose2d> get() = baseController.signal
    override fun update(reference: MotionState3<Pose2d>, currentState: Pose2d, elapsedSeconds: Double) {
        baseController.update(
            GlobalToBot.reference(reference, currentState),
            Pose2d.ZERO, elapsedSeconds
        )
    }

    override fun start() {
        baseController.start()
    }

    override fun stop() {
        baseController.stop()
    }
}