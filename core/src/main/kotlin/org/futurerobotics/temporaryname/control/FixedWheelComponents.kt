package org.futurerobotics.temporaryname.control

import koma.matrix.Matrix
import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.*

/**
 * A open-loop controller [FixedWheelDriveModel]; holonomic or non-holonomic,
 * that takes in [PoseMotion] as references, and produces [MotorVoltages] output.
 *
 * @param model the [FixedWheelDriveModel] to be used.
 */
class FixedWheelDriveOpenController(
    private val model: FixedWheelDriveModel
) : OpenController<Motion<Pose2d>, MotorVoltages>() {

    override fun getSignal(reference: Motion<Pose2d>, elapsedSeconds: Double): MotorVoltages {
        return model.getModeledVoltages(reference)
    }

    override fun start() {
        stop()
    }
}
//todo: state space???
/**
 * A [Observer] that uses motor poses and a drive [model] to estimate bot pose velocity.
 *
 * Maybe pass through a filter first.
 */
class FixedWheelPositionToVelocityObserver(private val model: FixedWheelDriveModel) :
    BaseObserver<MotorPositions, Any, Motion<Pose2d>>() {

    private var lastPositions: MotorPositions? = null
    private val cachedCurrentPositions: Matrix<Double>? = null
    override fun getState(measurement: MotorPositions, lastSignal: Any, elapsedSeconds: Double): Motion<Pose2d> {
        val lastPositions = lastPositions
        val velocity = if (lastPositions == null || elapsedSeconds.isNaN()) {
            this.lastPositions = measurement
            Pose2d.ZERO
        } else {
            val diff = measurement.zip(lastPositions) { cur, past -> (cur - past) / elapsedSeconds }
            model.getEstimatedVelocity(diff)
        }
        return ValueMotion(velocity,Pose2d.ZERO)
    }

    override fun start() {
        stop()
    }

    override fun stop() {
        invalidateState()
    }
}

/**
 * A [Observer] that uses estimated motor velocities and a drive [model] to estimate bot pose velocity.
 *
 * Maybe pass through a filter first.
 */
class FixedWheelVelocityToVelocityObserver(private val model: FixedWheelDriveModel) :
    BaseObserver<MotorVelocities, Any, Motion<Pose2d>>() {

    private val cachedCurrentPositions: Matrix<Double>? = null
    override fun getState(measurement: MotorVelocities, lastSignal: Any, elapsedSeconds: Double): Motion<Pose2d> {
        return ValueMotion(model.getEstimatedVelocity(measurement),Pose2d.ZERO)
    }

    override fun start() {
        stop()
    }

    override fun stop() {
        invalidateState()
    }
}
