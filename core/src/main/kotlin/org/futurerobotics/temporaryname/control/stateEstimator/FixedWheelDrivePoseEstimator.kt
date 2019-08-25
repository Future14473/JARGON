package org.futurerobotics.temporaryname.control.stateEstimator

import koma.matrix.Matrix
import org.futurerobotics.temporaryname.control.MotorPositions
import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.FixedWheelDriveModel
import org.futurerobotics.temporaryname.mechanics.Kinematics

class FixedWheelDrivePoseEstimator(private val model: FixedWheelDriveModel, initialState: Pose2d = Pose2d.ZERO) :
    StateEstimator<MotorPositions, Pose2d> {
    private var lastPositions: MotorPositions? = null
    private val cachedCurrentPositions: Matrix<Double>? = null
    override var currentState = initialState

    override fun update(measurement: MotorPositions, elapsedNanos: Long) {
        val lastPositions = lastPositions
        if (lastPositions == null || elapsedNanos == -1L) {
            this.lastPositions = measurement
            return
        }
        val diff = measurement.zip(lastPositions) { cur, past -> cur - past }
        val dpose = model.getEstimatedVelocity(diff)
        currentState = Kinematics.odometryUpdate(currentState, dpose)
    }

    override fun start() {
        lastPositions = false
    }

    override fun stop() {
        lastPositions = null
    }
}