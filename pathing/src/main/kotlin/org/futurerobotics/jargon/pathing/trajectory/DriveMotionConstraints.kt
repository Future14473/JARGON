package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.model.*
import org.futurerobotics.jargon.pathing.PathPoint
import kotlin.math.*

//to possibly do: constraints based on wheels

/** A [VelocityConstraint] that limit's each motor's angular speed. */
open class MaxMotorSpeed(
    private val maxes: Vec,
    private val interaction: MotorBotInteraction
) : VelocityConstraint {

    constructor(maxes: List<Double>, interaction: MotorBotInteraction) :
            this(maxes.toVec(), interaction)

    constructor(max: Double, interaction: MotorBotInteraction) :
            this(Vec(interaction.numMotors) { max }, interaction)

    override fun maxVelocity(point: PathPoint): Double =
        maxSpeedFromBotVelTransform(point, interaction.motorVelFromBotVel, maxes)
}

/** A constraint that limit's each motor's acceleration. */
open class MaxMotorAcceleration(
    private val maxes: Vec,
    private val interaction: MotorBotInteraction
) : AccelerationConstraint {

    constructor(maxes: List<Double>, interaction: MotorBotInteraction) :
            this(maxes.toVec(), interaction)

    constructor(max: Double, interaction: MotorBotInteraction) :
            this(Vec(interaction.numMotors) { max }, interaction)

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval =
        accelRangeFromBotAccelTransform(point, curVelocity, interaction.motorAccelFromBotAccel, maxes)
}

/**
 * A constraint that limits the max motor voltages on each wheel.
 *
 * A lot of fun math going around.
 */
class MaxMotorVoltage(
    private val maxes: Vec,
    private val interaction: MotorBotInteraction,
    private val motorVelocity: MotorVelocityDriveModel,
    private val motorFriction: MotorFrictionModel = ZeroMotorFrictionModel(interaction.numMotors)
) : AccelerationConstraint {

    init {
        require(interaction.numMotors == motorVelocity.numMotors) { "Num motors must match" }
        require(interaction.numMotors == maxes.size) { "Num motors must match" }
        require(interaction.numMotors == motorFriction.numMotors) { "Num motors must match" }
    }

    constructor(
        maxes: List<Double>,
        driveModel: DriveModel
    ) : this(maxes.toVec(), driveModel, driveModel, driveModel)

    constructor(
        max: Double,
        driveModel: DriveModel
    ) : this(Vec(driveModel.numMotors) { max }, driveModel, driveModel, driveModel)

    init {
        require(interaction.numMotors == motorVelocity.numMotors) {
            "Number of motors in interaction (${interaction.numMotors}) != number of motors in " +
                    "motorVelModel(${motorVelocity.numMotors})"
        }
    }

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        //lettuce assume that motorAccel = A*motorVel + B*volts + F*sign(motorVel)
        val vFma = motorVelocity.voltsFromMotorAccel
        val bv = (point.poseDeriv.vecRotated(-point.heading) * curVelocity).toVec()

        val maFmv = motorVelocity.motorAccelFromMotorVel
        val mv = interaction.motorVelFromBotVel * bv
        val maFf = motorFriction.motorAccelForMotorFriction
        val addend = vFma(maFmv * mv + maFf * sign(mv))

        val maFba = interaction.motorAccelFromBotAccel
        val vFba = vFma * maFba
        return accelRangeFromBotAccelTransform(
            point, curVelocity, vFba, maxes, addend
        )
    }
}

private fun rotationMatrix(angle: Double): Mat {
    val c = cos(angle)
    val s = sin(angle)
    return Mat(3, 3).apply {
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
    return Mat(3, 3).apply {
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
 * - `<<` indicates that the individual components of the left-hand vector each have a magnitude in the range
 * specified by the components on the right-hand side.
 * - `bot_vel` is the bot's velocity vector ([Pose2d.toVec])
 * - [mat] is a given matrix that transforms bot velocity into the constrained value
 * - [maxes] is a vector of allowable maximums of each of the components, which should all be positive
 *
 * This assumes all maximums are positive.
 */
fun maxSpeedFromBotVelTransform(
    point: PathPoint,
    mat: Mat,
    maxes: Vec
): Double {
    val factors = mat * point.poseDeriv.vecRotated(-point.heading).toVec()
    var result = Double.POSITIVE_INFINITY
    repeat(maxes.size) { i ->
        val max = maxes[i]
        val factor = factors[i]
        val curMax = abs(max / factor)
        result = min(result, curMax)
    }
    return result
}

/**
 * Calculates a range of allowable d^2s/dt^2, given that
 * ```
 * mat * bot_accel << +/-maxes + addend
 * ```
 * where:
 * - `<<` indicates that the individual components of the left-hand vector each have a magnitude in the range
 *  specified by the components on the right-hand side.
 * - [mat] is a given matrix that transforms bot acceleration into the constrained value
 * - `bot_accel` is the bot's acceleration vector ([Pose2d.toVec])
 * - [maxes] is a vector of allowable maximums of each of the components, which should all be positive
 * - [addend] is an optional vector of addends.
 *
 * and [curVelocity] is the current ds/dt.
 */
fun accelRangeFromBotAccelTransform(
    point: PathPoint,
    curVelocity: Double,
    mat: Mat,
    maxes: Vec,
    addend: Vec = Vec(maxes.size)
): Interval {
    val rot = rotationMatrix(-point.heading)
    val rotDeriv = rotationMatrixDeriv(-point.heading, -point.headingDeriv)
    val mults = mat * rot * point.poseDeriv.toVec()
    // Additional acceleration due to just motion (rotation and centripetal)
    // Similar to GlobalToBot.motion
    val fullAddend = addend - mat *
            (rot * (point.poseSecondDeriv * curVelocity.pow(2)).toVec() +
                    rotDeriv * (point.poseDeriv * curVelocity).toVec())
    var result = Interval.REAL
    repeat(maxes.size) { i ->
        val max = maxes[i]
        val mult = mults[i]
        val add = fullAddend[i]
        val interval = Interval.symmetricRegular(max / mult, add / mult)
        result = result.intersect(interval)
        if (result.isEmpty()) return Interval.EMPTY
    }
    return result
}
