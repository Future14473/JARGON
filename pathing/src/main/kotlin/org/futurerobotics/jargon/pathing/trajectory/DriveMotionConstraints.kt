package org.futurerobotics.jargon.pathing.trajectory

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Interval
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.pathing.PathPoint
import org.futurerobotics.jargon.util.repeatedList
import kotlin.math.abs
import kotlin.math.min


/**
 * Common components of all multiple-max based constraints.
 */
abstract class MultipleMaxesConstraint(maxes: List<Double>) : SingleConstraint {
    /** A list of maxes of some value*/
    protected val maxes: List<Double> = maxes.toList()

    init {
        require(maxes.all { it > 0 }) { "All maxes should be > 0; got ${maxes.joinToString()}" }
    }

    override fun otherIsRedundant(other: SingleConstraint): Boolean {
        if (other.javaClass != this.javaClass) return false
        other as MultipleMaxesConstraint
        return other.maxes.zip(this.maxes) { them, me -> them >= me }.all { it }
    }
}

/**
 * Common components of drive model based constraints, where maxes relate to individual wheels.
 *
 * @property driveModel the drive model used.*/
abstract class DriveModelConstraint(protected val driveModel: FixedDriveModel, maxes: List<Double>) :
    MultipleMaxesConstraint(maxes) {
    constructor(driveModel: FixedDriveModel, max: Double) :
            this(driveModel, repeatedList(driveModel.numWheels, max))
}

/**
 * Common base class for drive constraints related to velocity.
 *
 * @property driveModel the driveModel used.
 */
abstract class DriveMaxVelocityConstraint : DriveModelConstraint, VelocityConstraint {

    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override fun maxVelocity(point: PathPoint): Double {
        val factors = constrainedFromBotVel() * point.poseDeriv.toVector()
        var res = Double.POSITIVE_INFINITY
        maxes.forEachIndexed { i, max ->
            val factor = factors[i]
            val curMin = abs(max / factor)
            res = min(res, curMin)
        }
        return res
    }

    /** Gets a matrix that converts bot velocity into the constrained value's velocity. */
    protected abstract fun constrainedFromBotVel(): Mat
}

/** Common base class for drive constraints related to acceleration. */
abstract class DriveMaxAccelConstraint : DriveModelConstraint, AccelConstraint {

    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval {
        val constrainedFromBotAccel = constrainedFromBotAccel()
        val fromCentripetal = constrainedFromBotAccel * (point.poseSecondDeriv * curVelocity.squared()).toVector()
        val fromTangentialFactors = constrainedFromBotAccel * (point.poseDeriv.toVector())
        var res = Interval.REAL
        maxes.forEachIndexed { i, max ->
            val factor = fromTangentialFactors[i]
            val int = if (factor == 0.0) Interval.REAL
            else Interval.symmetricRegular(max / factor, -fromCentripetal[i] / factor)
            res = res.intersect(int)
        }
        return res
    }

    /** Gets a matrix that converts bot acceleration into the constrained item's acceleration. */
    protected abstract fun constrainedFromBotAccel(): Mat
}

private fun wheelToMotorSpeeds(wheelSpeeds: List<Double>, driveModel: FixedDriveModel): List<Double> =
    driveModel.wheels.zip(wheelSpeeds) { model, speed -> speed * model.motorVelPerWheelVel }

private fun wheelToMotorSpeeds(speed: Double, driveModel: FixedDriveModel): List<Double> =
    driveModel.wheels.map { model -> speed * model.motorVelPerWheelVel }

private fun wheelToMotorForces(wheelSpeeds: List<Double>, driveModel: FixedDriveModel): List<Double> =
    driveModel.wheels.zip(wheelSpeeds) { model, speed -> speed * model.motorTorquePerWheelTorque }

private fun wheelToMotorForces(speed: Double, driveModel: FixedDriveModel): List<Double> =
    driveModel.wheels.map { model -> speed * model.motorTorquePerWheelTorque }


/** A constraint that limit's each wheel's tangential speed. */
open class MaxWheelSpeed : DriveMaxVelocityConstraint {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override fun constrainedFromBotVel(): Mat = driveModel.wheelVelFromBotVel
}

/** A constraint that limit's each motor's speed. */
class MaxMotorSpeed : MaxWheelSpeed {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) :
            super(driveModel, wheelToMotorSpeeds(maxes, driveModel))

    constructor(driveModel: FixedDriveModel, max: Double) :
            super(driveModel, wheelToMotorSpeeds(max, driveModel))
}


/** A constraint that limit's each wheel's tangential acceleration (output acceleration). */
open class MaxWheelAccel : DriveMaxAccelConstraint {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override fun constrainedFromBotAccel(): Mat = driveModel.wheelAccelFromBotAccel
}

/** A constraint that limit's each motor's speed. */
class MaxMotorAccel : MaxWheelAccel {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) :
            super(driveModel, wheelToMotorSpeeds(maxes, driveModel))

    constructor(driveModel: FixedDriveModel, max: Double) :
            super(driveModel, wheelToMotorSpeeds(max, driveModel))
}


/** A constraint that limit's each the force of each wheel. */
open class MaxWheelForce : DriveMaxAccelConstraint {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override fun constrainedFromBotAccel(): Mat = driveModel.wheelForceFromBotAccel
}

/** A constraint that limit's each motor's force. */
class MaxMotorForce : MaxWheelForce {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) :
            super(driveModel, wheelToMotorForces(maxes, driveModel))

    constructor(driveModel: FixedDriveModel, max: Double) :
            super(driveModel, wheelToMotorForces(max, driveModel))
}

/** A constraint that limits the max motor voltages on each wheel. */
class MaxMotorVoltage : DriveMaxVelocityConstraint, MultipleConstraint {
    constructor(driveModel: FixedDriveModel, maxes: List<Double>) : super(driveModel, maxes)
    constructor(driveModel: FixedDriveModel, max: Double) : super(driveModel, max)

    override val velocityConstraints: Collection<VelocityConstraint> get() = listOf(this)
    override val accelConstraints: Collection<AccelConstraint> get() = listOf(VoltageAccelConstraint())

    private inner class VoltageAccelConstraint : AccelConstraint {
        val owner get() = this@MaxMotorVoltage
        override fun maxAccelRange(point: PathPoint, curVelocity: Double): Interval {
            val fromMotion = driveModel.voltsFromBotVel * (point.poseDeriv * curVelocity).toVector()
            //DRY?
            val fromCentripetal =
                driveModel.voltsFromBotAccel * (point.poseSecondDeriv * curVelocity.squared()).toVector()
            val fromTangentialFactors = driveModel.voltsFromBotAccel * point.poseDeriv.toVector()
            var res = Interval.REAL
            maxes.forEachIndexed { i, max ->
                val factor = fromTangentialFactors[i]
                val int = if (factor == 0.0) Interval.REAL
                else Interval.symmetricRegular(max / factor, -(fromMotion[i] + fromCentripetal[i]) / factor)
                res = res.intersect(int)
            }
            return res
        }

        override fun otherIsRedundant(other: SingleConstraint): Boolean =
            if (other !is VoltageAccelConstraint) false
            else owner.otherIsRedundant(other.owner)
    }

    override fun constrainedFromBotVel(): Mat = driveModel.voltsFromBotVel
}