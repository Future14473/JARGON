package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.*

/**
 * A open-loop controller [FixedWheelDriveModel]; holonomic or non-holonomic,
 * that takes in [MotionOnly] of Pose as references, and produces [MotorVoltages] output.
 *
 * For a better closed loop controllers, see (something that does not yet exist)
 *
 * Good to chain after controllers that map global references to local references.
 *
 * @param model the [FixedWheelDriveModel] to be used.
 */
class FixedWheelOpenController(
    private val model: FixedWheelDriveModel
) : OpenController<MotionOnly<Pose2d>, MotorVoltages>() {

    override fun getSignal(reference: MotionOnly<Pose2d>, elapsedSeconds: Double): MotorVoltages {
        return model.getModeledVoltages(reference)
    }

    override fun stop() {
    }
}

/**
 * A [Observer] that uses motor _positions_ and a drive [model] to estimate _bot_ pose velocity.
 *
 * Maybe pass through a filter first.
 */
class FixedWheelPositionToVelocityObserver(private val model: FixedWheelDriveModel) :
    BaseObserver<MotorPositions, Any, MotionOnly<Pose2d>>() {

    private var lastPositions: MotorPositions? = null
    override fun getState(measurement: MotorPositions, lastSignal: Any, elapsedSeconds: Double): MotionOnly<Pose2d> {
        val lastPositions = lastPositions
        val velocity = if (lastPositions == null || elapsedSeconds.isNaN()) {
            Pose2d.ZERO
        } else {
            val diff = measurement.zip(lastPositions) { cur, past -> (cur - past) / elapsedSeconds }
            model.getEstimatedVelocity(diff)
        }
        this.lastPositions = measurement
        return ValueMotionOnly(velocity, Pose2d.ZERO)
    }

    override fun start() {
        invalidateState()
    }

    override fun stop() {
    }
}

/**
 * A [Observer] that uses estimated motor _velocities_ and a drive [model] to estimate bot pose velocity.
 *
 * Maybe pass through a filter first.
 */
class FixedWheelVelocityToVelocityObserver(private val model: FixedWheelDriveModel) :
    BaseObserver<MotorVelocities, Any, MotionOnly<Pose2d>>() {

    override fun getState(measurement: MotorVelocities, lastSignal: Any, elapsedSeconds: Double): MotionOnly<Pose2d> {
        return ValueMotionOnly(model.getEstimatedVelocity(measurement), Pose2d.ZERO)
    }

    override fun start() {
        invalidateState()
    }

    override fun stop() {
    }
}
