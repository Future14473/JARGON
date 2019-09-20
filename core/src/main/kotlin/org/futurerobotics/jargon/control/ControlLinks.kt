@file:JvmName("ControlLinks")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.MotionOnly
import org.futurerobotics.jargon.mechanics.MotionState3

/**
 * A link that references global pose [MotionState3] and outputs pose [MotionOnly].
 *
 * The inputs of the controller will first be mapped from global reference to local reference, via
 * [GlobalToBotMotionController]. Observer will use [GlobalPoseObserver]
 *
 * @param controller the controller to use
 */
fun globalToBotMotionLink(controller: Controller<MotionState3<Pose2d>, Pose2d, MotionOnly<Pose2d>>):
        ControlLink<Pose2d, MotionState3<Pose2d>, MotionOnly<Pose2d>, MotionOnly<Pose2d>> {
    return ValueControlLink(GlobalToBotMotionController(controller), GlobalPoseObserver())
}
