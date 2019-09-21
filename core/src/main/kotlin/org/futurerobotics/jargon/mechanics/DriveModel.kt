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
     * The mass of this body
     */
    val mass: Double
    /**
     * The moment of inertia of this body.
     */
    val moi: Double
    /**
     * @return true if this robot can move in any direction AND rotate, else only move in one direction and rotate.
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

    /** The [FixedWheelModel]s of this drive model. */
    val wheels: List<FixedWheelModel> = wheels.toList()
    //useful matrices as linear transformations, used in calculations
    val voltsFromBotVel: Mat
    val voltsFromBotAccel: Mat
    val voltsFromWheelVel: Mat

    val botVelFromMotorVel: Mat
    val botAccelFromVolts: Mat

    val wheelVelFromBotVel: Mat
    val wheelAccelFromVolts: Mat

    val motorVelFromWheelVel: Mat
    val stallVolts: Vec
    init {
        //remember: a matrix is simply a linear transformation.
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

        voltsFromBotAccel = botAccelFromVolts.pinv() //least squares
        wheelVelFromBotVel = botForceFromWheelForce.T //turns out to be the same
        voltsFromWheelVel = pureDiag(wheels.map { it.voltsPerWheelVel })
        motorVelFromWheelVel = pureDiag(wheels.map { it.motorVelPerWheelVel })

        voltsFromBotVel = voltsFromWheelVel * wheelVelFromBotVel

        val wheelVelFromMotorVel = pureDiag(wheels.map { 1 / it.motorVelPerWheelVel })
        val botVelFromWheelVel = wheelVelFromBotVel.pinv()
        botVelFromMotorVel = botVelFromWheelVel * wheelVelFromMotorVel

        stallVolts = createVec(wheels.map { it.transmission.voltsForFriction })
        //wheel accel from wheel force
        val wheelAccelFromBotAccel = wheelVelFromBotVel
        wheelAccelFromVolts = wheelAccelFromBotAccel * botAccelFromVolts
    }

    val numWheels: Int get() = wheels.size
    /**
     * Gets the motor voltages corresponding modeled to drive at the given [MotionOnly] of Poses.
     * Used for a (partially) _open_ controller.
     */
    fun getModeledVoltages(motion: MotionOnly<Pose2d>): MotorVoltages {
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
     * Creates a drive model for a mecanum-like drive, with transmissions supplied in
     * [front left, front right, back left, back right] order, using NWU orientation.
     */
    @JvmStatic
    fun mecanumLike(
        mass: Double,
        moi: Double,
        transmissions: List<TransmissionModel>,
        wheelRadius: Double,
        horizontalRadius: Double,
        verticalRadius: Double
    ): HolonomicDriveModel {
        val wheels = listOf(
            FixedWheelModel(transmissions[0], Vector2d(verticalRadius, horizontalRadius), wheelRadius, -45 * degrees),
            FixedWheelModel(transmissions[1], Vector2d(verticalRadius, -horizontalRadius), wheelRadius, 45 * degrees),
            FixedWheelModel(transmissions[2], Vector2d(-verticalRadius, horizontalRadius), wheelRadius, 45 * degrees),
            FixedWheelModel(transmissions[3], Vector2d(-verticalRadius, -horizontalRadius), wheelRadius, -45 * degrees)
        )
        return HolonomicDriveModel(mass, moi, wheels)
    }
}