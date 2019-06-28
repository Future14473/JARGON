package org.futurerobotics.temporaryname.pathing.trajectory

import org.futurerobotics.temporaryname.math.Pose2d

/**
 * Represents a robot's field movement.
 *
 * @property pose The robot's current position/orientation relative to the world
 * @property velocity The robot's current velocity relative to the world
 */
data class FieldMotion(val pose: Pose2d, val velocity: Pose2d)