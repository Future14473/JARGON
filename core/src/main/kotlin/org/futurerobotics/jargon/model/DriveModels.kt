package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import kotlin.math.sqrt

/**
 * Gets a matrix which represents:
 * - the bot force from wheel force
 * - it's transpose is wheel velocity from bot velocity.
 */
private fun getWheelBotDynamicsMatrix(wheels: List<DriveWheelPosition>): Mat {
    val mat = Mat(3, wheels.size)
    wheels.forEachIndexed { i, wheel ->
        val (wheelVelPerXVel, wheelVelPerYVel) = wheel.orientation
        val wheelVelPerBotAngularVel = wheel.wheelVelPerBotAngularVel
        mat[0, i] = wheelVelPerXVel
        mat[1, i] = wheelVelPerYVel
        mat[2, i] = wheelVelPerBotAngularVel
    }
    return mat
}

/**
 * A drive model that only uses kinematics, but not mechanics/kinetics (only ([MotorBotInteraction])
 * This is used when there is no need to power the wheels.
 *
 * This only works with fixed wheels.
 *
 * @see DriveModel
 */
class KinematicsOnlyDriveModel(
    wheels: List<DriveWheelPosition>
) : MotorBotInteraction {

    override val numMotors: Int = wheels.size
    override val motorVelFromBotVel: Mat = getWheelBotDynamicsMatrix(wheels).T
    override val botVelFromMotorVel: Mat = motorVelFromBotVel.pinv()
}

/**
 * Models a vehicle/drive using using [PoweredDriveWheelModel]s, where the wheels are fixed in
 * place. This is suitable for mecanum, holonomic, differential, h-drives, and some other weird drives you might come
 * up with.
 *
 * Measurements in wheel position/moment of inertia should _ideally_ be relative to the center of mass.
 *
 * This implements [MotorVelocityDriveModel], [BotVelocityModel], [MotorBotInteraction], and [MotorFrictionModel].
 *
 * This model relies on the assumption that the wheels have perfect traction and are fixed to the drive body
 * (so moving one wheel moves the bot, which may affect the motion of other wheels). This may not be completely true
 * when wheels slip a lot or there are a lot of wheels. So while this model still be good for path planning, but a more
 * decoupled approach to controlling wheels (such as a separate controller per wheel) may work better in some cases,
 * compared to a controller that controls both wheels at once.
 *
 * Swerve drive is not yet supported.
 *
 * @param wheels the list of [PoweredDriveWheelModel]s relative to the center of mass of the robot
 * @param mass the mass of the bot
 * @param moi the moment of inertia of the bot
 *
 * @see KinematicsOnlyDriveModel
 */
class DriveModel(
    wheels: List<PoweredDriveWheelModel>,
    val mass: Double,
    val moi: Double
) : MotorBotInteraction, MotorFrictionModel,
    MotorVelocityDriveModel, BotVelocityModel {

    /**
     * Creates a [DriveModel].
     *
     * @param wheelTransmissions the [DriveWheelTransmission]s
     * @param power the [TorquePowerModel] used to power each wheel
     * @param mass the mass of hte bot
     * @param moi the moment of inertia of the bot
     */
    constructor(wheelTransmissions: List<DriveWheelTransmission>, power: TorquePowerModel, mass: Double, moi: Double)
            : this(wheelTransmissions.map { PoweredDriveWheelModel.fromTransmission(it, power) }, mass, moi)

    /**
     * Creates a [DriveModel].
     *
     * @param wheelPositions the [DriveWheelPosition]s
     * @param power the [PoweredWheelModel] used to power each wheel
     * @param mass the mass of hte bot
     * @param moi the moment of inertia of the bot
     */
    constructor(wheelPositions: List<DriveWheelPosition>, power: PoweredWheelModel, mass: Double, moi: Double)
            : this(wheelPositions.map { PoweredDriveWheelModel.fromPoweredWheel(it, power) }, mass, moi)

    init {
        require(wheels.isNotEmpty()) { "Drive model needs to have at least one wheel" }
        require(mass > 0) { "mass ($mass) should be > 0" }
        require(moi > 0) { "moi ($moi) should be > 0" }
    }

    private val wheelBotDynamicsMatrix: Mat = getWheelBotDynamicsMatrix(wheels)
    //interaction
    override val numMotors: Int = wheels.size
    override val motorVelFromBotVel: Mat = kotlin.run {
        val wheelVelFromBotVel = wheelBotDynamicsMatrix.T
        val motorVelFromWheelVel = wheels.mapToVec { it.inputVelPerOutputVel }.toDiagMat()
        motorVelFromWheelVel * wheelVelFromBotVel
    }
    override val botVelFromMotorVel: Mat = motorVelFromBotVel.pinv()
    //motor model
    private val botAccelFromWheelForce = kotlin.run {
        val botAccelFromBotForce = diagMatOf(1 / mass, 1 / mass, 1 / moi)
        val botForceFromWheelForce = wheelBotDynamicsMatrix
        botAccelFromBotForce * botForceFromWheelForce
    }
    //needed by motor velocity model
    override val botAccelFromVolts: Mat = kotlin.run {
        val wheelForceFromVolts = wheels.mapToVec { it.voltsPerForce }.toDiagMat()
        botAccelFromWheelForce * wheelForceFromVolts
    }

    //motor velocity
    override val voltsFromMotorVel: Mat =
        wheels.mapToVec { it.voltsPerVel * it.outputVelPerInputVel }
            .toDiagMat()

    override val motorAccelFromVolts: Mat = (motorVelFromBotVel) * botAccelFromVolts
    override val voltsFromMotorAccel: Mat = motorAccelFromVolts.pinv()

    override val motorAccelFromMotorVel: Mat = -motorAccelFromVolts * voltsFromMotorVel

    //bot velocity
    override val voltsFromBotAccel: Mat = botAccelFromVolts.pinv()
    override val voltsFromBotVel: Mat = voltsFromMotorVel * motorVelFromBotVel
    override val botAccelFromBotVel: Mat = -botAccelFromVolts * voltsFromBotVel

    //motor friction
    override val voltsForMotorFriction: Mat = wheels.mapToVec { it.additionalVoltsForFriction }.toDiagMat()

    override val motorAccelForMotorFriction: Mat = -motorAccelFromVolts * voltsForMotorFriction
}

object DriveModels {
    /**
     * Creates a drive model for a mecanum drive, with wheels in
     * `[front left, front right, back left, back right]` order. All wheels are oriented diagonally
     * forward, and the center of the bot is in the center of the wheels.
     *
     * The gear ratio will be multiplied by sqrt(2) since the wheels actually move diagonally.
     *
     * @param wheelRadius the wheel's radius
     * @param gearRatio the output:input gear ratio, used for both localization and power modeling. 1.0 if none.
     * @param horizontalDistance the distance between wheels on opposite sides of the bot
     * @param verticalDistance the distance between wheels on the same side of the bot
     */
    @JvmStatic
    fun mecanum(
        wheelRadius: Double,
        gearRatio: Double,
        horizontalDistance: Double,
        verticalDistance: Double
    ): List<DriveWheelTransmission> {
        val angles = arrayOf(
            -45 * deg, 45 * deg,
            45 * deg, -45 * deg
        )
        val verticalRadius = verticalDistance / 2
        val horizontalRadius = horizontalDistance / 2
        val locations = arrayOf(
            Vector2d(verticalRadius, horizontalRadius), Vector2d(verticalRadius, -horizontalRadius),
            Vector2d(-verticalRadius, horizontalRadius), Vector2d(-verticalRadius, -horizontalRadius)
        )
        val gearRatioSqrt2 = gearRatio * sqrt(2.0)
        return angles.zip(locations) { angle, location ->
            DriveWheelTransmission.of(location, angle, wheelRadius, gearRatioSqrt2)
        }
    }

    /**
     * Creates a drive model for a differential drive, with wheels in `[left, right]` order.
     * Wheels face forward, and the center of the two wheels is the center of the bot.
     *
     * @param wheelRadius the wheel's radius
     * @param gearRatio the output:input gear ratio, used for both localization and power modeling. 1.0 if none.
     * @param horizontalDistance the distance between wheels on opposite sides of the bot
     */
    @JvmStatic
    fun differential(
        wheelRadius: Double,
        gearRatio: Double,
        horizontalDistance: Double
    ): List<DriveWheelTransmission> {
        return arrayOf(0.5, -0.5).map { factor ->
            DriveWheelTransmission.of(
                location = Vector2d(0.0, factor * horizontalDistance),
                angle = 0.0,
                wheelRadius = wheelRadius,
                gearRatio = gearRatio
            )
        }
    }
}
