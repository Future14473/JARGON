package org.futurerobotics.temporaryname.mechanics

import koma.extensions.*
import koma.matrix.Matrix
import koma.sign
import koma.zeros
import org.futurerobotics.temporaryname.control.MotorVoltages
import org.futurerobotics.temporaryname.math.*
import org.futurerobotics.temporaryname.math.MathUnits.degrees
import org.futurerobotics.temporaryname.util.zipForEachIndexed

/**
 * Represents a model for drive; i.e. a body than an move and rotate via wheels or wheel like things.
 *
 * For a simpler model, these assume that the center of gravity correspond with the center of the robot (position 0,0)
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

    //TODO: What else goes here
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
    val wheels: List<FixedWheelModel>
) : DriveModel {
    init {
        require(mass >= 0) { "mass ($mass) should be >= 0" }
        require(moi >= 0) { "moi ($moi) should be >= 0" }
    }

    //question: which data representation do we really need
    private val botVelToVolts: Matrix<Double>
    private val botAccelToVolts: Matrix<Double> //this matrix * poseAccelMatrix = wheel torque matrix
    private val stallVolts: Matrix<Double>
    private val motorVelToBotVel: Matrix<Double>

    init {
        //remember: a matrix is simply a linear transformation.

        //turnContribution * bot ang vel = wheel tangent speed
        //turnContribution * wheel's force = wheel's contribution to bot torque. Yes, the other way around.
        val turnContributionVector = wheels.map { it.position cross it.orientation }

        val botForceToBotAccel = createDiag(1 / mass, 1 / mass, 1 / moi)
        val wheelForceToBotForce = zeros(3, wheels.size).also {
            wheels.zipForEachIndexed(turnContributionVector) { i, wheel, c ->
                // [fx, fy, c].transpose
                val (fx, fy) = wheel.orientation
                it[0, i] = fx
                it[1, i] = fy
                it[2, i] = c
            }
        }

        val voltsToWheelForce = createDiag(wheels.map { it.voltsPerForce })
        val voltsToBotAccel = botForceToBotAccel * wheelForceToBotForce * voltsToWheelForce

        botAccelToVolts = voltsToBotAccel.pinv() //least squares

        assert(botAccelToVolts.shape() == listOf(wheels.size, 3))

        val botVelToWheelVel = wheelForceToBotForce.transpose() //turns out to be the same
        val wheelVelToVolts =
            createDiag(wheels.map { 1 / it.voltsPerVelocity })

        botVelToVolts = wheelVelToVolts * botVelToWheelVel

        assert(botVelToVolts.shape() == listOf(wheels.size, 3))

        val motorVelToWheelVel = createDiag(wheels.map {
            it.motorAngVelPerVelocity
        })

        val wheelVelToBotVel = botVelToWheelVel.pinv()
        motorVelToBotVel = wheelVelToBotVel * motorVelToWheelVel

        stallVolts = create(wheels.map { it.transmission.stallVolts }).asColVector()
    }
    //todo: state-space stuff.
    /**
     * Gets the motor voltages corresponding modeled to drive at the given [velocity] and [acceleration]
     */
    fun getModeledVoltages(velocity: Pose2d, acceleration: Pose2d): MotorVoltages {
        val vels = botVelToVolts * velocity.toColumnVector()
        val accels = botAccelToVolts * acceleration.toColumnVector()
        return MotorVoltages((vels + accels + sign(vels) emul stallVolts).toList())
    }

    private val cachedMotorPosisitons: Matrix<Double> = zeros(wheels.size, 1)
    /**
     * Gets the estimated velocity based on the velocity in [motorPositions].
     *
     * This also gets the estimated _difference_ in _local_ pose given a difference in [motorPositions]
     */
    fun getEstimatedVelocity(motorPositions: List<Double>): Pose2d {
        require(motorPositions.size == motorVelToBotVel.numCols()) {
            "motorPositions should have same number of positions as this's wheels."
        }
        return Pose2d(motorVelToBotVel * cachedMotorPosisitons.setAllTo(motorPositions))
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
    override val isHolonomic: Boolean get() = true
}

/**
 * A drive model that can only move at a heading of 0 (Vector <1, 0>).
 * (Use the North-West-Up coordinate frame), but can still turn too.
 *
 * Making sure that the wheel configurations actually do this is left to the user.
 */
open class NonHolonomicDriveModel(
    mass: Double,
    moi: Double,
    wheels: List<FixedWheelModel>
) : FixedWheelDriveModel(mass, moi, wheels) {
    override val isHolonomic: Boolean get() = false
}

//TODO: Swerve

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