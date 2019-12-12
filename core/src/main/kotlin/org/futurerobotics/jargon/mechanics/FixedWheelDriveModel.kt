package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.util.zipForEachIndexed

/**
 * An implementation of all of [MotorVelocityModel], [BotVelocityModel], and [MotorBotVelInteraction]
 * that represents a body that uses ([WheelModel])s and bot mass and moment of inertia.
 *
 * **More importantly**, This model relies on an assumption tha the wheels have perfect traction and are perfectly
 * fixed to the drive body (so moving one wheel may affect all the others). One should consider using the values produced
 * by this model as only a starting point in creating a real model; although sometimes it may
 * perform well enough.
 *
 * @param wheelsAboutCenter the list of [WheelModel]s relative to the center _position_ of the robot
 * @param mass the mass of the bot
 * @param moi the moment of inertia of the bot
 * @param centerOfGravity the center of gravity of the bot relative to the center _position_. default <0, 0>.
 */
open class FixedWheelDriveModel
@JvmOverloads constructor(
    val mass: Double,
    val moi: Double,
    wheelsAboutCenter: List<WheelModel>,
    override val isHolonomic: Boolean,
    centerOfGravity: Vector2d = Vector2d.ZERO
) : MotorVelocityModel, BotVelocityModel, MotorBotVelInteraction {

    init {
        require(wheelsAboutCenter.isNotEmpty()) { "Drive model needs to have at least one wheel" }
        require(mass >= 0) { "mass ($mass) should be >= 0" }
        require(moi >= 0) { "moi ($moi) should be >= 0" }
    }

    private val wheels: List<WheelModel> = wheelsAboutCenter.map { (l, t) ->
        WheelModel(l.copy(locationAboutCenter = l.locationAboutCenter - centerOfGravity), t)
    }
    /**
     * The transmission models that this uses.
     */
    // todo: think about this
    val transmissions: List<TransmissionModel> = wheels.map { it.transmission }
    //private
    private val wheelBotDynamicsMatrix: Mat = kotlin.run {
        val velPerOmega = wheels.map { it.wheelPosition.tangentVelPerBotVel }
        zeroMat(3, wheels.size).also {
            wheels.zipForEachIndexed(velPerOmega) { i, (wheelPosition), c ->
                val (fx, fy) = wheelPosition.orientation
                it[0, i] = fx
                it[1, i] = fy
                it[2, i] = c
            }
        }
    }

    private val botVelFromComVel = idenMat(3).also {
        //right side =  <0,0,1> cross radius (radius = -centerOfGravity)
        it[0, 2] = centerOfGravity.y
        it[1, 2] = -centerOfGravity.x
    }
    private val comVelFromBotVel = botVelFromComVel.pinv()

    private val comAccelFromWheelForce by lazy {
        val comAccelFromBotForce = diagMat(1 / mass, 1 / mass, 1 / moi)
        comAccelFromBotForce * wheelBotDynamicsMatrix
    }

    private val comAccelFromVolts: Mat by lazy {
        val wheelForceFromVolts = diagMat(wheels.map { 1 / it.voltsPerOutputForce })
        comAccelFromWheelForce * wheelForceFromVolts
    }

    private val motorVelFromComVel: Mat by lazy {
        val wheelVelFromComVel = wheelBotDynamicsMatrix.T
        diagMat(wheels.map { it.motorVelPerOutputVel }) * wheelVelFromComVel
    }

    //motor velocity
    override val numMotors: Int get() = wheels.size

    override val voltsFromMotorVel: Mat by lazy { diagMat(wheels.map { it.transmission.motor.voltsPerAngVel }) }

    override val voltsFromMotorAccel: Mat by lazy { motorAccelFromVolts.pinv() }
    override val motorAccelFromVolts: Mat by lazy { motorVelFromComVel * comAccelFromVolts }

    override val motorAccelFromMotorVel: Mat by lazy { -motorAccelFromVolts * voltsFromMotorVel }

    override val motorAccelForMotorFriction: Mat by lazy {
        val wheelForceForMotorFriction = diagMat(wheels.map { it.forceForFriction })
        -motorVelFromComVel * comAccelFromWheelForce * wheelForceForMotorFriction
    }

    override val voltsForMotorFriction: Mat by lazy {
        voltsFromMotorAccel * -motorAccelForMotorFriction
    }
    //bot velocity
    override val voltsFromBotVel: Mat by lazy { voltsFromMotorVel * motorVelFromBotVel }

    override val voltsFromBotAccel: Mat by lazy { botAccelFromVolts.pinv() }
    override val botAccelFromVolts: Mat by lazy { botVelFromComVel * comAccelFromVolts }

    override val botAccelFromBotVel: Mat by lazy { -botAccelFromVolts * voltsFromBotVel }

    //interaction
    override val motorVelFromBotVel: Mat by lazy { motorVelFromComVel * comVelFromBotVel }
    override val botVelFromMotorVel: Mat by lazy { motorVelFromBotVel.pinv() }

    override val motorAccelFromBotAccel: Mat get() = motorVelFromBotVel
    override val botAccelFromMotorAccel: Mat get() = botVelFromMotorVel

    override val motorVelFromWheelVel: Mat by lazy { diagMat(wheels.map { it.motorVelPerOutputVel }) }
    override val wheelVelFromMotorVel: Mat by lazy { diagMat(wheels.map { 1 / it.motorVelPerOutputVel }) }

    companion object {

        /**
         * Creates a drive model for a mecanum-like drive, with wheels in
         * `[`front left, front right, back left, back right`]` order, using NWU orientation.
         */
        @JvmStatic
        @JvmOverloads
        fun mecanumLike(
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
                val location = WheelPosition(l, o, wheelRadius)
                WheelModel(location, transmission)
            }
            return FixedWheelDriveModel(mass, moi, wheels, true, centerOfGravity)
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
                WheelModel(wheelLocation, transmission)
            }
            return FixedWheelDriveModel(mass, moi, wheels, false, centerOfGravity)
        }
    }
}
