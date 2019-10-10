package org.futurerobotics.jargon.simulation

import org.junit.jupiter.api.Test

internal class ASimulation {
    @Test
    fun `a simulation`() {
//        val driveModel = DriveModels.differential(
//            5 * lbs, 10.0, TransmissionModel.fromTorqueLosses(
//                DcMotorModel.fromMotorData(
//                    12.0, 10 * lbf * ft, 4.0, 1000 * rev / min, 0.1
//                ),
//                1.0, 1.0, 0.9
//            ), 2 * `in`, 6 * `in`
//        )
//        val simulatedFixedDrive = SimulatedFixedDrive(
//            driveModel,
//            FixedDriveModelPerturber(
//                0.1, FixedWheelModelPerturb(
//                    0.1, TransmissionModelPerturb(
//                        0.1, DcMotorModelPerturb(0.1)
//                    )
//                )
//            ),
//            Random("a simulation".hashCode().toLong()),
//            eye(2) * 0.5,
//            eye(2) * 0.5,
//            0.01
//        )
//        val system = buildBlockSystem {
//            val motors = SimulatedDriveInput(simulatedFixedDrive).add()
//            val trajs = ExternalQueue<Trajectory>()
//            motors.input<Double>(1) connectFrom loopTime
//            val voltageInput = motors.input<List<Double>>()
//            val motorPositions = motors<List<Double>>(0)
//            val motorVelocities = motors<List<Double>>(1)
//            val trajQueue = trajs.add()
//        }
    }
}