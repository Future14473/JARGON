package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.GlobalToRelative
import org.futurerobotics.temporaryname.mechanics.PoseMotion
import org.futurerobotics.temporaryname.mechanics.PoseMotionState

/**
 * A wrapper around another controller that first the reference from global to relative first.
 */
class GlobalToRelativePoseController(
    private val baseController: Controller<PoseMotionState, Pose2d, PoseMotion>
) : Controller<PoseMotionState, Pose2d, PoseMotion> {

    override val signal: PoseMotion get() = baseController.signal
    override fun update(reference: PoseMotionState, currentState: Pose2d, elapsedSeconds: Double) {
        //let target always be Zero, and the state given is the error.
        baseController.update(GlobalToRelative.reference(reference, currentState), Pose2d.ZERO, elapsedSeconds)
    }

    override fun start() {
        baseController.start()
    }

    override fun stop() {
        baseController.stop()
    }
}