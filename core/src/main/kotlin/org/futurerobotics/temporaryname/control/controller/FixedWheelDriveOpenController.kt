package org.futurerobotics.temporaryname.control.controller

import org.futurerobotics.temporaryname.control.MotorVoltages
import org.futurerobotics.temporaryname.math.Pose2d
import org.futurerobotics.temporaryname.mechanics.FixedWheelDriveModel

/**
 * A open-loop controller [FixedWheelDriveModel]; holonomic or non-holonomic,
 * that takes in [Pose2d] velocity and acceleration as references, and produces [MotorVoltages] output.
 *
 * @param model the [FixedWheelDriveModel] to be used.
 */
class FixedWheelDriveOpenController(
    private val model: FixedWheelDriveModel
) : OpenController<Pose2d, Pose2d, MotorVoltages>() {
    override fun reset() {}

    override fun process(reference: Pose2d, referenceDeriv: Pose2d?, elapsedNanos: Long): MotorVoltages {
        return model.getModeledVoltages(reference, referenceDeriv ?: Pose2d.ZERO)
    }
}