package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.DriveModel
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.util.repeatedList
import kotlin.math.*

/**
 * Common components of drive model based constraints, where maxes relate to individual wheels.
 *
 * @property driveModel the drive model used.*/
abstract class WheelDriveMotionConstraint(protected val driveModel: DriveModel, maxes: List<Double>) :
    MultipleMaxBasedConstraint(maxes), MotionConstraint {

    constructor(driveModel: DriveModel, max: Double) :
            this(driveModel, repeatedList(driveModel.numWheels, max))
}

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
    maxes: List<Double>
): Double {
    val factors = mat * point.poseDeriv.vecRotated(-point.heading).toVec()
    var res = Double.POSITIVE_INFINITY
    maxes.forEachIndexed { i, max ->
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
    maxes: List<Double>,
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
    maxes.forEachIndexed { i, max ->
        val mult = mults[i]
        val add = fullAddend[i]
        val interval = Interval.symmetricRegular(max / mult, add / mult)
        res = res.intersect(interval)
        if (res.isEmpty()) return@forEachIndexed
    }
    return res
}

private fun List<Double>.wheelToMotorVel(driveModel: DriveModel): List<Double> =
    (driveModel.motorVelFromWheelVel * this.toVec()).asList()

private fun Double.wheelToMotorVel(driveModel: DriveModel): List<Double> =
    (driveModel.motorVelFromWheelVel * repeatedList(driveModel.numWheels, this).toVec()).asList()

/** A [VelConstraint] that limit's each motor's angular velocity. */
open class MaxMotorSpeed : WheelDriveMotionConstraint, VelConstraint {

    constructor(driveModel: DriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: DriveModel, max: Double) : super(driveModel, max)

    override fun maxVelocity(point: PathPoint): Double =
        maxSpeedFromBotVelTransform(point, driveModel.motorVelFromBotVel, maxes)
}

/** A constraint that limit's each wheel's tangential speed. */
open class MaxWheelTangentialSpeed : MaxMotorSpeed {

    constructor(driveModel: DriveModel, maxes: List<Double>) :
            super(driveModel, maxes.wheelToMotorVel(driveModel))

    constructor(driveModel: DriveModel, max: Double) :
            super(driveModel, max.wheelToMotorVel(driveModel))
}

/** A constraint that limit's each motor's speed. */
open class MaxMotorAccel : WheelDriveMotionConstraint, AccelConstraint {

    constructor(driveModel: DriveModel, maxes: List<Double>) :
            super(driveModel, maxes.wheelToMotorVel(driveModel))

    constructor(driveModel: DriveModel, max: Double) :
            super(driveModel, max.wheelToMotorVel(driveModel))

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval =
        accelRangeFromBotAccelTransform(point, curVelocity, driveModel.motorAccelFromBotAccel, maxes)
}

/** A constraint that limit's each wheel's tangential acceleration (output acceleration). */
open class MaxWheelTangentialAccel : MaxMotorAccel, AccelConstraint {

    constructor(driveModel: DriveModel, maxes: List<Double>) :
            super(driveModel, maxes.wheelToMotorVel(driveModel))

    constructor(driveModel: DriveModel, max: Double) :
            super(driveModel, max.wheelToMotorVel(driveModel))
}

/** A constraint that limits the max motor voltages on each wheel. */
class MaxMotorVoltage : WheelDriveMotionConstraint, AccelConstraint {

    constructor(driveModel: DriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: DriveModel, max: Double) : super(driveModel, max)

    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        //lettuce assume that motorAccel = A*motorVel + B*volts + F*sign(motorVel)
        val maFmv = driveModel.motorAccelFromMotorVel
        val vFma = driveModel.voltsFromMotorAccel
        val maFba = driveModel.motorAccelFromBotAccel
        val vFba = vFma * maFba
        val bv = (point.poseDeriv.vecRotated(-point.heading) * curVelocity).toVec()
        val mv = driveModel.motorVelFromBotVel * bv
        val maFf = driveModel.motorAccelForFriction
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
class MaxMotorTorque : WheelDriveMotionConstraint, AccelConstraint {

    constructor(driveModel: DriveModel, maxes: List<Double>) : super(driveModel, maxes)

    constructor(driveModel: DriveModel, max: Double) : super(driveModel, max)

    private val torqueFromVolts = diagMat(driveModel.transmissions.map { 1 / it.motor.voltsPerTorque })
    override fun accelRange(point: PathPoint, curVelocity: Double): Interval {
        val tFma = torqueFromVolts * driveModel.voltsFromMotorAccel
        val tFba = tFma * driveModel.motorAccelFromBotAccel
        val bv = (point.poseDeriv.vecRotated(-point.heading)).toVec()
        val mv = driveModel.motorVelFromBotVel * bv
        val maFf = driveModel.motorAccelForFriction
        val addend = tFma(maFf * sign(mv))
        return accelRangeFromBotAccelTransform(
            point, curVelocity, tFba, maxes, addend
        )
    }
}
