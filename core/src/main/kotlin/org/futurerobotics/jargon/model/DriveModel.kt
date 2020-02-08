package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d

/**
 * A drive model framed on analyzing on how motor voltages affect each motor's velocity, with possibly
 * interactions between separate motors (applying a voltage to one motor may also move another).
 *
 * A desired bot velocity/acceleration will first need to be convrted to motor velocities/accelerations first,
 * using [MotorBotInteraction].
 *
 * Models should be continuous.
 *
 * @see BotVelocityModel
 * @see DriveModel
 */
interface MotorVelocityDriveModel : MotorBotInteraction {

    /**
     * Transforms motor velocities into the voltages needed to sustain that speed.
     *
     * This can be due to the motors acting as generators, hence producing a voltage that needs
     * to be counterbalanced, and/or the contribution of other frictional forces.
     *
     * Non-diagonal entries may or may not be 0 (there are interactions between motors).
     *
     * Adding this with voltages obtained from [voltsFromMotorAccel] and possibly from [MotorFrictionModel] gets the
     * modeled voltage required.
     */
    val voltsFromMotorVel: Mat
    /**
     * Transforms motor acceleration into the voltages needed to produce that acceleration, given no initial velocity.
     *
     * This may factor in possible frictional forces.
     *
     * Non-diagonal entries may or may not be 0 (there are interactions between motors).
     *
     * Adding this with voltages obtained from [voltsFromMotorVel] and possibly from [MotorFrictionModel] gets the
     * voltage required.
     * @see motorAccelFromVolts
     */
    val voltsFromMotorAccel: Mat
    /**
     * Transforms motor voltage into expected motor acceleration, given no initial velocity.
     *
     * This is used for observation/prediction.
     *
     * Non-diagonal entries may or may not be 0 (there are interactions between motors).
     *
     * This should be the (pseudo) inverse of [voltsFromMotorAccel].
     * @see voltsFromMotorAccel
     */
    val motorAccelFromVolts: Mat
    /**
     * Gets the expected motor (de)acceleration due to existing motor velocities. This
     * can be caused by the motor acting as a generator hence producing a voltage that slows it down, and/or other
     * frictional forces that are dependent on the bot's velocity.
     *
     * This should (usually) have **negative** diagonal entries.
     *
     * Non-diagonal entries may or may not be 0 (there are interactions between motors).
     */
    val motorAccelFromMotorVel: Mat
}

/**
 * A drive model framed on analyzing on how motor voltages affect the bot's overall velocity,
 * ignoring the individual motor velocities.
 *
 * For drives with only a few wheels, [MotorVelocityDriveModel] may be better.
 *
 * @see MotorVelocityDriveModel
 * @see DriveModel
 */
interface BotVelocityModel {

    /** The number of motors. */
    val numMotors: Int
    /**
     * Transforms bot pose velocity (see [Pose2d.toVec]) into the expected motor volts needed to sustain that speed,
     * with no acceleration.
     *
     * This can be due to motors acting as generators, hence producing a voltage that needs to be counterbalanced;
     * and/or the contribution of other frictional forces.
     *
     * This can then be added with [voltsFromBotAccel] to get the actual modeled volts.
     */
    val voltsFromBotVel: Mat
    /**
     * Transform bot pose acceleration (see [Pose2d.toVec]) into needed motor volts to produce that acceleration,
     * from zero initial speed.
     *
     * This can then be added with [voltsFromBotVel] to get the actual modeled volts.
     * @see botAccelFromVolts
     */
    val voltsFromBotAccel: Mat
    /**
     * Transform motor volts into the expected bot pose acceleration vector (see [Pose2d.toVec]), assuming no
     * friction and no initial speed.
     *
     * This should be the (pseudo)inverse of [voltsFromBotAccel].
     *
     * @see voltsFromBotAccel
     */
    val botAccelFromVolts: Mat
    /**
     * Transform the currents bot pose velocity into the expected bot pose (de)acceleration.
     *
     * This can be due to motors acting as generators and hence producing a voltage that slows them down, or the
     * contribution
     * of other frictional forces.
     *
     * Should (usually) have ***negative*** diagonal entries.
     * @see voltsFromBotAccel
     */
    val botAccelFromBotVel: Mat
}

/**
 * Represents additional modeling of frictional forces based on motor velocities and voltages, on a drive train.
 *
 * @see DriveModel
 */
interface MotorFrictionModel {

    /**
     * The number of motors.
     */
    val numMotors: Int
    /**
     * Represents the _constant_ components of frictional forces; Transforms a vector of _signs_ of the motor's
     * velocities into how that affects all motor's acceleration.
     *
     * Should (usually) have ***negative*** diagonal entries.
     *
     * May be 0 to assume no friction.
     *
     * This should (usually) have negative diagonal entries.
     *
     * @see voltsForMotorFriction
     */
    val motorAccelForMotorFriction: Mat
    /**
     * Transforms a vector of _signs_ of the motor's
     * velocities into the amount of volts needed to add to compensate for it.
     *
     * Should (usually) have ***positive*** diagonal entries.
     *
     * May be 0 to assume no friction.
     *
     * @see voltsForMotorFriction
     */
    val voltsForMotorFriction: Mat
}

/**
 * Represents the transforming to and from motor velocities/accelerations into bot velocities/accelerations.
 *
 * @see KinematicsOnlyDriveModel
 * @see DriveModel
 */
interface MotorBotInteraction {

    /** The number of motors/wheels. */
    val numMotors: Int
    /**
     * Transforms a bot velocity vector (see [Pose2d.toVec]) into motor angular velocities.
     * @see botVelFromMotorVel
     */
    val motorVelFromBotVel: Mat
    /**
     * Transforms motor angular velocities into a bot velocity vector (see [Pose2d.toVec]), possibly least squares.
     * @see motorVelFromBotVel
     */
    val botVelFromMotorVel: Mat
    /**
     * Transforms a bot acceleration vector (see [Pose2d.toVec]) into motor angular acceleration.
     *
     * This is usually the same as [motorVelFromBotVel].
     * @see botAccelFromMotorAccel
     */
    val motorAccelFromBotAccel: Mat get() = motorVelFromBotVel
    /**
     * Transforms motor angular acceleration into a bot acceleration vector (see [Pose2d.toVec]),
     * possibly least squares (see [Pose2d.toVec]).
     *
     * This is usually the same as [motorAccelFromBotAccel].
     * @see motorAccelFromBotAccel
     */
    val botAccelFromMotorAccel: Mat get() = botVelFromMotorVel
}
