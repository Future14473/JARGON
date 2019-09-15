@file:JvmName("ControlLinks")

package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.Motion
import org.futurerobotics.temporaryname.mechanics.State

/**
 * A link that references global pose [State] and outputs pose [Motion].
 *
 * The inputs of the controller will first be mapped from global reference to local reference, via
 * [GlobalToRelativePoseController]. Observer will use [GlobalPoseObserver]
 *
 * @param controller the controller to use
 */
fun globalToRelativeMotionLink(controller: Controller<State<Pose2d>, Pose2d, Motion<Pose2d>>):
        ControlLink<Pose2d, State<Pose2d>, Motion<Pose2d>, Motion<Pose2d>> {
    return ValueControlLink(GlobalToRelativePoseController(controller), GlobalPoseObserver())
}
