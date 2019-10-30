package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.degrees
import org.futurerobotics.jargon.util.asUnmodifiableList
import org.futurerobotics.jargon.util.toImmutableList
import org.futurerobotics.jargon.util.zipForEachIndexed

/**
 * Represents a model for fixed-wheel drive, i.e., a body than an move and rotate via wheels or wheel-like things.
 *
 * This assumes that the center of gravity corresponds with the center of the robot (position 0,0)
 *
 * The standard is to use North-west-up orientation: +x is forward, +y is left, angles are CCW.
 */
//todo: think about for swerve drives, interface segregation -- different linearity.
interface DriveModel {

    /** The fancy math word that means this can move in any direction AND rotate independently of each other. */
    val isHolonomic: Boolean

    /**
     * A list of [TransmissionModel]s used by this bot.
     */
    val transmissions: List<TransmissionModel>

    /** The number of wheels. */
    val numWheels: Int
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
     * Transforms a bot velocity vector (see [Pose2d.toVec]) into the expected motor volts to sustain that speed
     * with no acceleration.
     * @see botVelFromVolts
     */
    val voltsFromBotVel: Mat
    /**
     * Transforms motor volts into the expected constant bot velocity (see [Pose2d.toVec]), when the voltage is
     * sustained, with least squares error.
     * @see [voltsFromBotVel]
     */
    val botVelFromVolts: Mat
    /**
     * Transforms a bot acceleration vector (see [Pose2d.toVec]) into motor angular acceleration.
     * @see botAccelFromMotorAccel
     * @see botVelFromMotorVel
     */
    val motorAccelFromBotAccel: Mat
    /**
     * Transforms motor angular acceleration into a bot acceleration vector (see [Pose2d.toVec]), with least squares
     * error.
     * @see motorAccelFromBotAccel
     * @see motorVelFromBotVel
     */
    val botAccelFromMotorAccel: Mat
    /**
     * Transform a bot acceleration vector (see [Pose2d.toVec]) into needed motor volts, assuming no friction and
     * no initial speed.
     *
     * This can then be added with [voltsFromBotVel] and friction to get the actual volts.
     * @see botAccelFromVolts
     */
    val voltsFromBotAccel: Mat
    /**
     * Transform motor volts into a bot acceleration vector (see [Pose2d.toVec]), assuming no
     * friction and no initial speed.
     *
     * @see voltsFromBotAccel
     */
    val botAccelFromVolts: Mat
    /**
     * Gets the expected motor (de)acceleration due to existing motor velocities; caused by the motor
     * acting as a generator hence producing a voltage, slowing it down, and assuming no friction.
     *
     * This should (usually) have negative diagonal entries.
     *
     * @see motorVelFromMotorAccel
     */
    val motorAccelFromMotorVel: Mat
    /**
     * Inverse of [motorAccelFromMotorVel]. Only here to preserve sacred symmetry.
     */
    val motorVelFromMotorAccel: Mat
    /**
     * Transforms motor velocities to generated volts; due to the motor acting as a generator.
     */
    val voltsFromMotorVel: Mat
    /**
     * Inverse of [voltsFromMotorVel]. Only here to preserve sacred symmetry.
     */
    val motorVelFromVolts: Mat
    /**
     * Transforms motor acceleration into required motor volts, given no initial velocity. This can
     * then be added with [voltsFromMotorVel] and friction compensation to get the actual motor volts.
     *
     * Note that this makes the assumption that wheels are perfectly glued to the floor and so that turning one motor
     * will interact with each other, which is sometimes not the case in real life (sort of for
     * differential-like drives, not for holonomic drives). So, only use this model as a starting point.
     *
     * @see motorAccelFromVolts
     */
    val voltsFromMotorAccel: Mat
    /**
     * Transforms motor voltage into expected motor acceleration, given no initial velocity.
     *
     * Note that this makes the assumption that wheels are perfectly glued to the floor and so that turning one motor
     * will interact with each other, which is sometimes not the case in real life (sort of for
     * differential-like drives, not for holonomic drives). So, only use this model as a starting point.
     *
     * @see voltsFromMotorAccel
     */
    val motorAccelFromVolts: Mat
    /**
     * Transforms a vector of (signs of the angular velocity) to how that affects the motor's acceleration.
     *
     * Should (usually) have negative diagonal entries.
     *
     * May be 0 to assume no/negligible/not considered friction.
     */
    val motorAccelForFriction: Mat
    /**
     * Transforms a vector of (signs of the angular velocity) into the required additional volts to overcome it.
     *
     * Should (usually) have negative diagonal entries.
     *
     * May be 0 to assume no/negligible/not considered friction.
     */
    val voltsForFriction: Mat
    /**
     * Transforms motor velocities into wheel (tangential) velocities
     */
    val wheelVelFromMotorVel: Mat

    /**
     * Transforms a wheel (tangential) velocities into motor velocities
     */
    val motorVelFromWheelVel: Mat
}

/**
 * An implementation of a [DriveModel] that represents a body that moves around via wheels that cannot
 * change location or orientation ([FixedWheelModel])s, and assuming that the model is perfect (which it often is not).
 *
 * **More importantly**, This model relies on an assumption tha the wheels have perfect traction and are perfectly
 * fixed to the drive body (so moving one wheel may affect all the others). One should consider using the values produced
 * by this model as only a starting point in creating a real model; although sometimes it may
 * perform well enough (for drives with few wheels).
 *
 * @param wheels the list of [FixedWheelModel]s
 */
open class FixedWheelDriveModel(
    val mass: Double,
    val moi: Double,
    wheels: List<FixedWheelModel>,
    final override val isHolonomic: Boolean
) : DriveModel {

    init {
        require(wheels.isNotEmpty()) { "Drive model needs to have at least one wheel" }
        require(mass >= 0) { "mass ($mass) should be >= 0" }
        require(moi >= 0) { "moi ($moi) should be >= 0" }
    }

    /** the [FixedWheelModel]s that this uses. */
    val wheels: List<FixedWheelModel> = wheels.toImmutableList()
    override val transmissions: List<TransmissionModel>
        get() = wheels.map { it.transmission }.asUnmodifiableList()
    override val numWheels: Int get() = wheels.size
    private val botForceFromWheelForce by lazy {
        val turnContributionVector = wheels.map { it.position cross it.orientation }
        zeroMat(3, wheels.size).also {
            wheels.zipForEachIndexed(turnContributionVector) { i, wheel, c ->
                // [fx, fy, c].T
                val (fx, fy) = wheel.orientation
                it[0, i] = fx
                it[1, i] = fy
                it[2, i] = c
            }
        }
    }

    override val motorVelFromBotVel: Mat by lazy {
        val wheelVelFromBotVel = botForceFromWheelForce.T  //turns out to be the same
        diagMat(wheels.map { it.motorVelPerOutputVel }) * wheelVelFromBotVel
    }
    override val botVelFromMotorVel: Mat by lazy { motorVelFromBotVel.pinv() }
    override val voltsFromBotVel: Mat by lazy { voltsFromMotorVel * motorVelFromBotVel }
    override val botVelFromVolts: Mat by lazy { voltsFromBotVel.pinv() }
    override val motorAccelFromBotAccel: Mat get() = motorVelFromBotVel
    override val botAccelFromMotorAccel: Mat get() = botVelFromMotorVel
    override val voltsFromBotAccel: Mat by lazy { botAccelFromVolts.pinv() }
    override val botAccelFromVolts: Mat by lazy {
        val botAccelFromBotForce = diagMat(1 / mass, 1 / mass, 1 / moi)
        val wheelForceFromVolts = diagMat(wheels.map { 1 / it.voltsPerOutputForce })
        botAccelFromBotForce * botForceFromWheelForce * wheelForceFromVolts
    }
    override val motorAccelFromMotorVel: Mat by lazy { -motorAccelFromVolts * voltsFromMotorVel }
    override val motorVelFromMotorAccel: Mat by lazy { motorAccelFromMotorVel.inv() }
    override val voltsFromMotorVel: Mat by lazy { diagMat(transmissions.map { it.motor.voltsPerAngVel }) }
    override val motorVelFromVolts: Mat by lazy { diagMat(transmissions.map { 1 / it.motor.voltsPerAngVel }) }
    override val voltsFromMotorAccel: Mat by lazy { motorAccelFromVolts.pinv() }
    override val motorAccelFromVolts: Mat by lazy { motorAccelFromBotAccel * botAccelFromVolts }
    override val motorAccelForFriction: Mat by lazy {
        -motorAccelFromVolts * diagMat(transmissions.map { it.voltsForFriction })
    }
    override val voltsForFriction: Mat by lazy { voltsFromMotorAccel * motorAccelForFriction }
    override val motorVelFromWheelVel: Mat by lazy { diagMat(wheels.map { it.motorVelPerOutputVel }) }
    override val wheelVelFromMotorVel: Mat by lazy { diagMat(wheels.map { 1 / it.motorVelPerOutputVel }) }
}

/**
 * Utility for creating common drive models.
 */
object DriveModels {

    /**
     * Creates a drive model for a mecanum-like drive, with wheels in
     * [front left, front right, back left, back right] order, using NWU orientation.
     */
    @JvmStatic
    fun mecanumLike(
        mass: Double,
        moi: Double,
        transmission: TransmissionModel,
        wheelRadius: Double,
        horizontalRadius: Double,
        verticalRadius: Double
    ): FixedWheelDriveModel {
        val wheels = listOf(
            FixedWheelModel.fromWheelAngle(
                transmission,
                Vector2d(verticalRadius, horizontalRadius),
                wheelRadius,
                -44.99 * degrees //deviate to prevent problems with singular matrices.
            ),
            FixedWheelModel.fromWheelAngle(
                transmission,
                Vector2d(verticalRadius, -horizontalRadius),
                wheelRadius,
                45 * degrees
            ),
            FixedWheelModel.fromWheelAngle(
                transmission,
                Vector2d(-verticalRadius, horizontalRadius),
                wheelRadius,
                45 * degrees
            ),
            FixedWheelModel.fromWheelAngle(
                transmission,
                Vector2d(-verticalRadius, -horizontalRadius),
                wheelRadius,
                -45 * degrees
            )
        )
        return FixedWheelDriveModel(mass, moi, wheels, true)
    }

    /**
     * Creates a drive model for a differential supplied, wheels in [left, right] order, using NWU orientation.
     */
    @JvmStatic
    fun differential(
        mass: Double,
        moi: Double,
        transmission: TransmissionModel,
        wheelRadius: Double,
        horizontalRadius: Double
    ): FixedWheelDriveModel {
        val wheels = listOf(
            FixedWheelModel.fromWheelAngle(transmission, Vector2d(0.0, horizontalRadius), wheelRadius, 0.0),
            FixedWheelModel.fromWheelAngle(transmission, Vector2d(0.0, -horizontalRadius), wheelRadius, 0.0)
        )
        return FixedWheelDriveModel(mass, moi, wheels, false)
    }
}
