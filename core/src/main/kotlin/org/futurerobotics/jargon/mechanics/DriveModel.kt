package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.degrees
import org.futurerobotics.jargon.util.zipForEachIndexed

/**
 * Represents a model for drive, i.e. a body than an move and rotate via wheels or wheel-like things.
 *
 * For a simpler model, this assumes that the center of gravity corresponds with the center of the robot (position 0,0)
 *
 * Keep in mind all math in this library uses North-west-up orientation: +x is forward, +y is left, angles are CCW.
 */
interface DriveModel {

    /**
     * The mass of this body, or Double.NaN if not known for testing purposes only.
     */
    val mass: Double
    /**
     * The mass of this body, or Double.NaN if not known for testing purposes only.
     */
    val moi: Double
    /**
     * @return true if this robot can move in any direction AND rotate independently of each other.
     */
    val isHolonomic: Boolean
}

/**
 * A base implementation of a [DriveModel] that represents a body that moves around via wheels that cannot
 * change location or orientation.
 *
 * @param wheels the list of [FixedWheelModel]s
 */
abstract class FixedWheelDriveModel(
    final override val mass: Double,
    final override val moi: Double,
    wheels: List<FixedWheelModel>
) : DriveModel {

    init {
        require(mass >= 0) { "mass ($mass) should be >= 0" }
        require(moi >= 0) { "moi ($moi) should be >= 0" }
    }

    private val wheels: List<FixedWheelModel> = wheels.toList()

    /** The number of wheels. */
    val numWheels: Int get() = wheels.size
    /** Transforms motor volts into bot acceleration, assuming a current velocity of 0; least squares. */
    val botAccelFromVolts: Mat
    /** Transforms a bot velocity vector into wheel velocities. */
    val wheelVelFromBotVel: Mat

    init {
        //turnContribution * bot ang vel = wheel tangent speed
        //turnContribution * wheel's force = wheel's contribution to bot torque. Yes, the other way around.

        val turnContributionVector = wheels.map { it.position cross it.orientation }
        val botAccelFromBotForce = pureDiag(1 / mass, 1 / mass, 1 / moi)
        val botForceFromWheelForce = zeros(3, wheels.size).also {
            wheels.zipForEachIndexed(turnContributionVector) { i, wheel, c ->
                // [fx, fy, c].T
                val (fx, fy) = wheel.orientation
                it[0, i] = fx
                it[1, i] = fy
                it[2, i] = c
            }
        }
        val wheelForceFromVolts = pureDiag(wheels.map { 1 / it.motorVoltsPerOutputForce })
        botAccelFromVolts = botAccelFromBotForce * botForceFromWheelForce * wheelForceFromVolts

        wheelVelFromBotVel = botForceFromWheelForce.T //turns out to be the same

    }

    /** The matrix that transforms wheel velocities into wheel volts, with no acceleration. */
    val voltsFromWheelVel: Mat by lazy { pureDiag(wheels.map { it.voltsPerWheelVel }) }
    /** The matrix that transforms a pose velocity vector into the expected volts, with no acceleration. */
    val voltsFromBotVel: Mat by lazy { voltsFromWheelVel * wheelVelFromBotVel }
    /**
     * The matrix that transform a pose acceleration vector into the expected volts, with no speed.
     *
     * This is linear with [voltsFromBotVel]
     */
    val voltsFromBotAccel: Mat by lazy { botAccelFromVolts.pinv() }

    /** Transforms motor velocities into bot velocities; least squares. */
    val botVelFromMotorVel: Mat by lazy {
        val wheelVelFromMotorVel = pureDiag(wheels.map { 1 / it.motorVelPerWheelVel })
        val botVelFromWheelVel = wheelVelFromBotVel.pinv()
        botVelFromWheelVel * wheelVelFromMotorVel
    }
    /** Transforms wheel velocities into motor velocities. */
    val motorVelFromWheelVel: Mat by lazy { pureDiag(wheels.map { it.motorVelPerWheelVel }) }
    /**
     * Gets the expected wheel acceleration given the wheel volts; least squares.
     *
     * This assumes that no wheels slip and they are all interlinked.
     */
    val wheelAccelFromVolts: Mat by lazy { wheelVelFromBotVel * botAccelFromVolts }
    /** The amount of volts needed to overcome frictional forces, ignoring acceleration. */
    val stallVolts: Vec by lazy { createVec(wheels.map { it.transmission.voltsForFriction }) }

    /**
     * Gets the motor voltages corresponding modeled to drive at the given [MotionOnly] of Poses.
     * Used for a (partially) _open_ controller.
     */
    fun getModeledVoltages(motion: MotionOnly<Pose2d>): List<Double> {
        val (v, a) = motion
        val vels = voltsFromBotVel * v.toVector()
        val accels = voltsFromBotAccel * a.toVector()
        return (vels + accels + sign(vels) emul stallVolts).toList()
    }

    /**
     * Gets the estimated velocity based on the velocity in [motorPositions].
     *
     * This also gets the estimated _difference_ in _local_ pose given a difference in [motorPositions]
     */
    fun getEstimatedVelocity(motorPositions: List<Double>): Pose2d {
        require(motorPositions.size == botVelFromMotorVel.columnDimension) {
            "motorPositions should have same number of positions as this's wheels."
        }
        return Pose2d(botVelFromMotorVel * motorPositions.toDoubleArray())
    }
}

/**
 * A drive model that both turn and move in any direction.
 *
 * Making sure that the wheel configurations actually are holonomic is left to the user.
 */
open class HolonomicDriveModel(
    mass: Double,
    moi: Double,
    wheels: List<FixedWheelModel>
) : FixedWheelDriveModel(mass, moi, wheels) {

    final override val isHolonomic: Boolean get() = true
}

/**
 * A drive model that can only move at a heading of 0 (Vector <1, 0>).
 * (Use the North-West-Up coordinate frame), but can still turn too.
 *
 * Making sure that the wheel configurations are actually non-holonomic do this is left to the user,
 * or else the model may perform better than expected.
 */
open class NonHolonomicDriveModel(
    mass: Double,
    moi: Double,
    wheels: List<FixedWheelModel>
) : FixedWheelDriveModel(mass, moi, wheels) {

    final override val isHolonomic: Boolean get() = false
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
    ): HolonomicDriveModel {
        val wheels = listOf(
            FixedWheelModel(transmission, Vector2d(verticalRadius, horizontalRadius), wheelRadius, -45 * degrees),
            FixedWheelModel(transmission, Vector2d(verticalRadius, -horizontalRadius), wheelRadius, 45 * degrees),
            FixedWheelModel(transmission, Vector2d(-verticalRadius, horizontalRadius), wheelRadius, 45 * degrees),
            FixedWheelModel(transmission, Vector2d(-verticalRadius, -horizontalRadius), wheelRadius, -45 * degrees)
        )
        return HolonomicDriveModel(mass, moi, wheels)
    }

    /**
     * Creates a drive model for a differential supplied, wheels in [left, right] order, using NWU orientation.
     */
    fun differential(
        mass: Double,
        moi: Double,
        transmission: TransmissionModel,
        wheelRadius: Double,
        horizontalRadius: Double
    ): NonHolonomicDriveModel {
        val wheels = listOf(
            FixedWheelModel(transmission, Vector2d(0.0, horizontalRadius), wheelRadius, 0.0),
            FixedWheelModel(transmission, Vector2d(0.0, -horizontalRadius), wheelRadius, 0.0)
        )
        return NonHolonomicDriveModel(mass, moi, wheels)
    }
}