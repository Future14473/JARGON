@file:JvmName("ControlLinks")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.Motion
import org.futurerobotics.jargon.mechanics.State

/**
 * A link that references global pose [State] and outputs pose [Motion].
 *
 * The inputs of the controller will first be mapped from global reference to local reference, via
 * [GlobalToBotMotionController]. Observer will use [GlobalPoseObserver]
 *
 * @param controller the controller to use
 */
fun globalToBotMotionLink(controller: Controller<State<Pose2d>, Pose2d, Motion<Pose2d>>):
        ControlLink<Pose2d, State<Pose2d>, Motion<Pose2d>, Motion<Pose2d>> {
    return ValueControlLink(GlobalToBotMotionController(controller), GlobalPoseObserver())
}
