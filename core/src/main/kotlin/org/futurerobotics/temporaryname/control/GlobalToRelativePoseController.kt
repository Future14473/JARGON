package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.FieldToBot
import org.futurerobotics.temporaryname.mechanics.Motion
import org.futurerobotics.temporaryname.mechanics.State

/**
 * A wrapper around another controller that first maps reference from global to relative first.
 *
 * The current state supplied will always be [Pose2d.ZERO], and reference moves around.
 */
class GlobalToRelativePoseController(
    private val baseController: Controller<State<Pose2d>, Pose2d, Motion<Pose2d>>
) : Controller<State<Pose2d>, Pose2d, Motion<Pose2d>> {

    override val signal: Motion<Pose2d> get() = baseController.signal
    override fun update(reference: State<Pose2d>, currentState: Pose2d, elapsedSeconds: Double) {
        baseController.update(FieldToBot.reference(reference, currentState), Pose2d.ZERO, elapsedSeconds)
    }

    override fun start() {
        baseController.start()
    }

    override fun stop() {
        baseController.stop()
    }
}