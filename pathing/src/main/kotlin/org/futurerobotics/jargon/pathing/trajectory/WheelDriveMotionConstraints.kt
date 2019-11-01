package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.MotorBotVelInteraction
import org.futurerobotics.jargon.mechanics.MotorModel
import org.futurerobotics.jargon.mechanics.MotorVelocityModel
import org.futurerobotics.jargon.pathing.PathPoint
import kotlin.math.*

private fun rotationMatrix(angle: Double): Mat {
    val c = cos(angle)
    val s = sin(angle)
    return zeroMat(3, 3).apply {
        this[0, 0] = c
        this[0, 1] = -s
        this[1, 0] = s
        this[1, 1] = c
        this[2, 2] = 1.0
    }
}

private fun rotationMatrixDeriv(angle: Double, angleDeriv: Double): Mat {
    val c = cos(angle) * angleDeriv
    val s = sin(angle) * angleDeriv
    return zeroMat(3, 3).apply {
        this[0, 0] = -s
        this[0, 1] = -c
        this[1, 0] = c
        this[1, 1] = -s
    }
}

/**
 * Calculates the maximum allowable ds/dt given that
 * ```
 * mat * bot_vel << +/-maxes
 * ```
 * where:
 * - `<<` indicates that the individual elements of the left-hand vector is within the ranges specified by the
 *  right-hand side.
 * - [mat] is a given matrix that transforms bot velocity into the constrained value
 * - `bot_vel` is the bot's velocity vector (see [Pose2d.toVec])
 *
 * This assumes all maximums are positive.
 */
fun maxSpeedFromBotVelTransform(
    point: PathPoint,
    mat: Mat,
    maxes: Vec
): Double {
    val factors = mat * point.poseDeriv.vecRotated(-point.heading).toVec()
    var res = Double.POSITIVE_INFINITY
    repeat(maxes.size) { i ->
        val max = maxes[i]
        val factor = factors[i]
        val curMax = abs(max / factor)
        res = min(res, curMax)
    }
    return res
}

/**
 * Calculates the maximum allowable ds/dt given that
 * ```
 * mat * bot_accel << +/-maxes + addend
 * ```
 * where:
 * - `<<` indicates that the individual elements of the left-hand vector is within the ranges specified by the
 *  right-hand side.
 * - [mat] is a given matrix that transforms bot acceleration into the constrained value
 * - `bot_accel` is the bot's acceleration vector (see [Pose2d.toVec])
 * - [maxes] is a given list of maximums
 * - [addend] is added to the interval.
 *
 * This assumes all maximums are positive.
 */
fun accelRangeFromBotAccelTransform(
    point: PathPoint,
    curVelocity: Double,
    mat: Mat,
    maxes: Vec,
    addend: Vec = zeroVec(maxes.size)
): Interval {
    val rot = rotationMatrix(-point.heading)
    val rotDeriv = rotationMatrixDeriv(-point.heading, -point.headingDeriv)
    val mults = mat * rot * point.poseDeriv.toVec()
    // Additional acceleration due to just motion (rotation and centripetal)
    // Similar to GlobalToBot.motion
    val fullAddend = addend - mat *
            (rot * (point.poseSecondDeriv * curVelocity.pow(2)).toVec() +
                    rotDeriv * (point.poseDeriv * curVelocity).toVec())
    var res = Interval.REAL
    repeat(maxes.size) { i ->
        val max = maxes[i]
        val mult = mults[i]
        val add = fullAddend[i]
        val interval = Interval.symmetricRegular(max / mult, add / mult)
        res = res.intersect(interval)
        if (res.isEmpty()) return@repeat
    }
    return res
}

private fun Vec.wheelToMotorVel(interaction: MotorBotVelInteraction): Vec =
    interaction.motorVelFromWheelVel * this

private fun Double.wheelToMotorVel(interaction: MotorBotVelInteraction): Vec =
    interaction.motorVelFromWheelVel * genVec(interaction.numMotors) { this }

/** A [VelConstraint] that limit's each motor's angular velocity. */
open class MaxMotorSpeed protected constructor(
    private val interaction: MotorBotVelInteraction,
    private val maxes: Vec
) : VelConstraint {

    constructor(motorVelModel: MotorBotVelInteraction, maxes: List<Double>) :
            this(motorVelModel, maxes.toVec())

    constructor(motorVelModel: MotorBotVelInteraction, max: Double) :
            this(motorVelModel, genVec(motorVelModel.numMotors) { max })

    override fun maxVelocity(point: PathPoint): Double =
        maxSpeedFromBotVelTransform(point, interaction.motorVelFromBotVel, maxes)
}

/** A constraint that limit's each wheel's tangential speed. */
open class MaxWheelTangentialSpeed protected constructor(
    interaction: MotorBotVelInteraction,
    maxes: Vec
) : MaxMotorSpeed(interaction, maxes.wheelToMotorVel(interaction)) {

    constructor(interaction: MotorBotVelInteraction, maxes: List<Double>) :
            this(interaction, maxes.toVec())

    constructor(interaction: MotorBotVelInteraction, max: Double) :
            this(interaction, genVec(interaction.numMotors) { max })
}

/** A constraint that limit's each motor's speed. */
open class MaxMotorAccel protected constructor(
    private val interaction: MotorBotVelInteraction,
    private val maxes: Vec
) : AccelConstraint {

    constructor(interaction: MotorBotVelInteraction, maxes: List<Double>) : this(interaction, maxes.toVec())

    constructor(interaction: MotorBotVelInteraction, max: Double) : this(interaction, max.wheelToMotorVel(interaction))

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval =
        accelRangeFromBotAccelTransform(point, curVelocity, interaction.motorAccelFromBotAccel, maxes)
}

/** A constraint that limit's each wheel's tangential acceleration (output acceleration). */
open class MaxWheelTangentialAccel protected constructor(
    interaction: MotorBotVelInteraction,
    maxes: Vec
) : MaxMotorAccel(interaction, maxes.wheelToMotorVel(interaction)) {

    constructor(interaction: MotorBotVelInteraction, maxes: List<Double>) :
            this(interaction, maxes.toVec())

    constructor(interaction: MotorBotVelInteraction, max: Double) :
            this(interaction, genVec(interaction.numMotors) { max })
}

/** A constraint that limits the max motor voltages on each wheel. */
class MaxMotorVoltage private constructor(
    private val interaction: MotorBotVelInteraction,
    private val motorVelModel: MotorVelocityModel,
    private val maxes: Vec
) : AccelConstraint {

    constructor(interaction: MotorBotVelInteraction, motorVelModel: MotorVelocityModel, maxes: List<Double>) :
            this(interaction, motorVelModel, maxes.toVec())

    constructor(interaction: MotorBotVelInteraction, motorVelModel: MotorVelocityModel, max: Double) :
            this(interaction, motorVelModel, genVec(interaction.numMotors) { max })

    init {
        require(interaction.numMotors == motorVelModel.numMotors) {
            "Number of motors in interaction ($interaction) != number of motors in " +
                    "motorVelModel(${motorVelModel.numMotors})"
        }
    }

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        //lettuce assume that motorAccel = A*motorVel + B*volts + F*sign(motorVel)
        val maFmv = motorVelModel.motorAccelFromMotorVel
        val vFma = motorVelModel.voltsFromMotorAccel
        val maFba = interaction.motorAccelFromBotAccel
        val vFba = vFma * maFba
        val bv = (point.poseDeriv.vecRotated(-point.heading) * curVelocity).toVec()
        val mv = interaction.motorVelFromBotVel * bv
        val maFf = motorVelModel.motorAccelForMotorFriction
        val addend = vFma(maFmv * mv + maFf * sign(mv))

        return accelRangeFromBotAccelTransform(
            point, curVelocity, vFba, maxes, addend
        )
    }
}

/**
 * A constraint that limit's each motor's force.
 *
 * This assumes that the friction due to the _motor_ internally
 * is negligible due to the friction due to the transmission/wheels/bot.
 * */
class MaxMotorTorque private constructor(
    motorModels: List<MotorModel>,
    private val interaction: MotorBotVelInteraction,
    private val motorVelModel: MotorVelocityModel,
    private val maxes: Vec
) : AccelConstraint {

    constructor(
        motorModels: List<MotorModel>,
        interaction: MotorBotVelInteraction,
        motorVelModel: MotorVelocityModel,
        maxes: List<Double>
    ) : this(motorModels, interaction, motorVelModel, maxes.toVec())

    constructor(
        motorModels: List<MotorModel>,
        interaction: MotorBotVelInteraction,
        motorVelModel: MotorVelocityModel,
        max: Double
    ) : this(motorModels, interaction, motorVelModel, genVec(interaction.numMotors) { max })

    init {
        require(interaction.numMotors == motorVelModel.numMotors) {
            "Number of motors in interaction ($interaction) != number of motors in " +
                    "motorVelModel(${motorVelModel.numMotors})"
        }
    }

    private val torqueFromVolts = diagMat(motorModels.map { 1 / it.voltsPerTorque })
    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        val tFma = torqueFromVolts * motorVelModel.voltsFromMotorAccel
        val tFba = tFma * interaction.motorAccelFromBotAccel
        val bv = (point.poseDeriv.vecRotated(-point.heading)).toVec()
        val mv = interaction.motorVelFromBotVel * bv
        val maFf = motorVelModel.motorAccelForMotorFriction
        val addend = tFma(maFf * sign(mv))
        return accelRangeFromBotAccelTransform(
            point, curVelocity, tFba, maxes, addend
        )
    }
}
