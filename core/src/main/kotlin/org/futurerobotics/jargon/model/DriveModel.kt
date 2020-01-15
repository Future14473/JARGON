package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d

/**
 * Represents a model that is framed analyzing how motor voltages affects the motor's velocity, with possibly
 * some interactions between separate motors (applying a voltage to one motor may also move another).
 *
 * Models should be _continuous_.
 *
 * @see BotVelocityControllingModel
 */
interface MotorVelocityControllingModel {

    /**
     * The number of motors in this model.
     */
    val numMotors: Int
    /**
     * Transforms motor velocities into the voltages needed to keep it at that speed.
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
 * Represents additional modeling of frictional forces based on motor velocity/voltage.
 */
interface MotorFrictionModel {

    /**
     * The number of motors.
     */
    val numMotors: Int
    /**
     * Represents the _constant_ components of frictional forces; Transforms a vector of _signs_ of the motor's
     * velocities into how that affects the motor's acceleration.
     *
     * For non_constant components that depend on the motor velocity, see [motorAccelFromMotorVel].
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
     * Represents the _constant_ components of frictional forces; Transforms a vector of _signs_ of the motor's
     * velocities into the amount of volts needed to add to compensate for it.
     *
     * For non_constant components that depend on the motor velocity, see [motorAccelFromMotorVel].
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
 * Represents a drive model that is framed analyzing how motor voltages affects the bot's velocity, ignoring
 * motor's individual velocities.
 *
 * Models should be _continuous_.
 *
 * For drives with a few number of wheels, [MotorVelocityControllingModel] may be better.
 *
 * @see MotorVelocityControllingModel
 */
interface BotVelocityControllingModel {

    /** The number of motors. */
    val numMotors: Int
    /**
     * Transforms bot pose velocity (see [Pose2d.toVec]) into the expected motor volts to sustain that speed,
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
//No easy way to calculate this while still being linear, besides empirically
//    /**
//     * Gets the expected constant component of the bot acceleration, from transforming a vector of the _signs_ of the
//     * bot velocity (since constant friction is usually related to the direction of motion).
//     *
//     * For _non_ constant friction/friction-like forces, see [botAccelFromBotVel]
//     * @see voltsForBotFriction
//     */
//    val botAccelForBotFriction: Mat
//    /**
//     * Gets the expected amount of volts needed to be added to compensate for _constant_ friction, from transforming a
//     * vector of the _signs_ of the bot velocity (since constant friction is usually related to the direction of
//     * motion).
//     *
//     * For _non_ constant friction/friction-like forces, see [botAccelFromBotVel] and [voltsFromBotAccel]
//     * @see botAccelForBotFriction
//     */
//    val voltsForBotFriction: Mat
}

/**
 * Represents the transforming of motor velocities/accelerations into bot velocities/accelerations, and vice versa.
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
     * Transforms motor angular velocities into a bot velocity vector (see [Pose2d.toVec]), with least squares error.
     * @see motorVelFromBotVel
     */
    val botVelFromMotorVel: Mat
    /**
     * Transforms a bot acceleration vector (see [Pose2d.toVec]) into motor angular acceleration.
     * @see botAccelFromMotorAccel
     * @see botVelFromMotorVel
     */
    val motorAccelFromBotAccel: Mat get() = motorVelFromBotVel
    /**
     * Transforms motor angular acceleration into a bot acceleration vector (see [Pose2d.toVec]), with least squares
     * error.
     * @see motorAccelFromBotAccel
     * @see motorVelFromBotVel
     */
    val botAccelFromMotorAccel: Mat get() = botVelFromMotorVel
}

/**
 * Includes [botVelFromMotorAndGyroVel], which is used for odometry/localization in combination with gyroscope data.
 */
interface MotorBotGyroInteraction {

    /**
     * The number of motors.
     */
    val numMotors: Int
    /**
     * Transforms motor angular velocities vector, _plus_ an entry for measured bot angular velocity,
     * into expected bot velocity.
     *
     * This is usually calculated by taking the (pseudo) inverse of a `motor vel from bot vel` matrix plus an
     * extra [0, 0, 1] row (corresponding to the fact that bot angular velocity equals bot angular velocity).
     */
    val botVelFromMotorAndGyroVel: Mat
}

/**
 * Represents the interaction between wheel velocities and motor velocities.
 */
interface MotorWheelInteraction {

    /**
     * The number of motors/wheels.
     */
    val numMotors: Int
    /**
     * Transforms motor velocities into wheel (tangential) velocities
     */
    val wheelVelFromMotorVel: Mat

    /**
     * Transforms a wheel (tangential) velocities into motor velocities
     */
    val motorVelFromWheelVel: Mat
}
