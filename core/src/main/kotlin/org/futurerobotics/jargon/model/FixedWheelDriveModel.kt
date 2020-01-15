package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.util.zipForEachIndexed
import kotlin.math.sqrt

/**
 * From wheels about a center point, gets a matrix which both represents
 * the bot applied force from wheel applied force, and it's transpose is wheel velocity from bot velocity.
 */
private fun getWheelBotDynamicsMatrix(wheelsAboutCenter: List<FixedWheelModel>): Mat {
    val velPerOmega = wheelsAboutCenter.map { it.wheelPosition.tangentVelPerBotVel }
    return zeroMat(3, wheelsAboutCenter.size).also {
        wheelsAboutCenter.zipForEachIndexed(velPerOmega) { i, (wheelPosition), c ->
            val (fx, fy) = wheelPosition.orientation
            it[0, i] = fx
            it[1, i] = fy
            it[2, i] = c
        }
    }
}

/**
 * A drive model that only concerns with kinematics (velocity/acceleration), [MotorBotInteraction],
 * [MotorBotGyroInteraction], not mechanics/kinetics (actual forces). This is used when mass/inertia is not
 * known or not required.
 *
 * This only works with fixed wheels.
 *
 * @see FixedWheelDriveModel
 */
class KinematicsOnlyDriveModel(
    wheelsAboutCenter: List<FixedWheelModel>
) : MotorBotInteraction, MotorBotGyroInteraction {

    private val wheelsAboutCenter = wheelsAboutCenter.toList()
    private val wheelBotDynamicsMatrix: Mat = getWheelBotDynamicsMatrix(wheelsAboutCenter)

    override val numMotors: Int
        get() = wheelsAboutCenter.size
    override val motorVelFromBotVel: Mat
        get() = TODO("not implemented")
    override val botVelFromMotorVel: Mat
        get() = TODO("not implemented")
    override val botVelFromMotorAndGyroVel: Mat
        get() = TODO("not implemented")
}

/**
 * An implementation of [MotorVelocityControllingModel], [BotVelocityControllingModel], [MotorBotInteraction],
 * [MotorWheelInteraction], [MotorFrictionModel], that models a robot with [FixedWheelModel] (motors fixed in place
 * on the robot). This is valid for mecanum, holonomic, differential, h-drives, and other weird drives you might come
 * up with.
 *
 * **More importantly**, This model relies on an assumption that the wheels have perfect traction and are perfectly
 * fixed to the drive body (so moving one wheel may affect all the others). This may not be completely true when
 * wheels slip a lot.
 *
 * Todo: add documentation on alternatives to consider in this case.
 *
 * @param mass the mass of the bot
 * @param moi the moment of inertia of the bot
 * @param wheelsAboutCenter the list of [FixedWheelModel]s relative to the _center position_ of the robot
 * @param centerOfGravity the _center of mass_ of the bot relative to the center _position_. default <0, 0>.
 */
class FixedWheelDriveModel
@JvmOverloads constructor(
    val mass: Double,
    val moi: Double,
    wheelsAboutCenter: List<FixedWheelModel>,
    centerOfGravity: Vector2d = Vector2d.ZERO
) : MotorVelocityControllingModel, BotVelocityControllingModel,
    MotorBotInteraction, MotorWheelInteraction, MotorBotGyroInteraction,
    MotorFrictionModel {

    init {
        require(wheelsAboutCenter.isNotEmpty()) { "Drive model needs to have at least one wheel" }
        require(mass > 0) { "mass ($mass) should be > 0" }
        require(moi > 0) { "moi ($moi) should be > 0" }
    }

    //com = center of mass
    private val wheelsAboutCom: List<FixedWheelModel> = wheelsAboutCenter.map { (l, t) ->
        FixedWheelModel(l.copy(locationAboutCenter = l.locationAboutCenter - centerOfGravity), t)
    }
    private val wheelBotDynamicsMatrix: Mat = getWheelBotDynamicsMatrix(wheelsAboutCom)

    private val botVelFromComVel = idenMat(3).also {
        //insert <0,0,1> cross radius (radius = -centerOfGravity)
        it[0, 2] = centerOfGravity.y
        it[1, 2] = -centerOfGravity.x
    }
    private val comVelFromBotVel = botVelFromComVel.pinv()

    private val comAccelFromWheelForce = kotlin.run {
        val comAccelFromBotForce = diagMat(1 / mass, 1 / mass, 1 / moi)
        comAccelFromBotForce * wheelBotDynamicsMatrix
    }

    private val comAccelFromVolts: Mat = kotlin.run {
        val wheelForceFromVolts = diagMat(wheelsAboutCom.map { 1 / it.voltsPerOutputForce })
        comAccelFromWheelForce * wheelForceFromVolts
    }

    private val motorVelFromComVel: Mat = kotlin.run {
        val wheelVelFromComVel = wheelBotDynamicsMatrix.T
        diagMat(wheelsAboutCom.map { it.motorVelPerOutputVel }) * wheelVelFromComVel
    }

    override val numMotors: Int get() = wheelsAboutCom.size
    //motor velocity
    //from motor models.
    override val voltsFromMotorVel: Mat = diagMat(wheelsAboutCom.map { it.transmission.motor.voltsPerAngVel })
    //from com.
    override val motorAccelFromVolts: Mat = motorVelFromComVel * comAccelFromVolts
    override val voltsFromMotorAccel: Mat = motorAccelFromVolts.pinv()

    override val motorAccelFromMotorVel: Mat = -motorAccelFromVolts * voltsFromMotorVel

    override val motorAccelForMotorFriction: Mat = kotlin.run {
        val wheelForceForMotorFriction = diagMat(wheelsAboutCom.map { it.forceForFriction })
        -motorVelFromComVel * comAccelFromWheelForce * wheelForceForMotorFriction
    }

    override val voltsForMotorFriction: Mat = voltsFromMotorAccel * -motorAccelForMotorFriction
    //bot velocity
    override val botAccelFromVolts: Mat = botVelFromComVel * comAccelFromVolts
    override val voltsFromBotAccel: Mat = botAccelFromVolts.pinv()

    //interaction
    override val motorVelFromBotVel: Mat = motorVelFromComVel * comVelFromBotVel
    override val botVelFromMotorVel: Mat = motorVelFromBotVel.pinv()

    override val voltsFromBotVel: Mat = voltsFromMotorVel * motorVelFromBotVel
    override val botAccelFromBotVel: Mat = -botAccelFromVolts * voltsFromBotVel

    override val motorVelFromWheelVel: Mat = diagMat(wheelsAboutCom.map { it.motorVelPerOutputVel })
    override val wheelVelFromMotorVel: Mat = diagMat(wheelsAboutCom.map { 1 / it.motorVelPerOutputVel })

    override val botVelFromMotorAndGyroVel: Mat = kotlin.run {
        val motorVelAndGyroFromBotVel = concatCol(motorVelFromBotVel, Mat(0, 0, 1))
        motorVelAndGyroFromBotVel.pinv()
    }

    companion object {

        /**
         * Creates a drive model for a mecanum drive, with wheels in
         * `[`front left, front right, back left, back right`]` order, using NWU orientation.
         *
         * The wheel radius will be multiplied by a factor of sqrt(2) since they are actually effectively facing
         * diagonally.
         */
        @JvmStatic
        @JvmOverloads
        fun mecanum(
            mass: Double,
            moi: Double,
            transmission: TransmissionModel,
            wheelRadius: Double,
            horizontalRadius: Double,
            verticalRadius: Double,
            centerOfGravity: Vector2d = Vector2d.ZERO
        ): FixedWheelDriveModel {
            val orientations = listOf(
                -45 * deg, 44.99 * deg, //problems with singular matrices...
                45 * deg, -45 * deg
            ).map { Vector2d.polar(1.0, it) }
            val locations = listOf(
                Vector2d(verticalRadius, horizontalRadius), Vector2d(verticalRadius, -horizontalRadius),
                Vector2d(-verticalRadius, horizontalRadius), Vector2d(-verticalRadius, -horizontalRadius)
            )
            val wheels = orientations.zip(locations) { o, l ->
                val location = WheelPosition(l, o, wheelRadius * sqrt(2.0))
                FixedWheelModel(location, transmission)
            }
            return FixedWheelDriveModel(mass, moi, wheels, centerOfGravity)
        }

        /**
         * Creates a drive model for a differential supplied, wheels in [left, right] order, using NWU orientation.
         */
        @JvmStatic
        @JvmOverloads
        fun differential(
            mass: Double,
            moi: Double,
            transmission: TransmissionModel,
            wheelRadius: Double,
            horizontalRadius: Double,
            centerOfGravity: Vector2d = Vector2d.ZERO
        ): FixedWheelDriveModel {
            val wheels = listOf(1, -1).map {
                val wheelLocation = WheelPosition(
                    Vector2d(0.0, it * horizontalRadius),
                    Vector2d.polar(1.0, 0.0), wheelRadius
                )
                FixedWheelModel(wheelLocation, transmission)
            }
            return FixedWheelDriveModel(mass, moi, wheels, centerOfGravity)
        }
    }
}
