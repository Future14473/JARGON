package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.model.FixedWheelDriveModel
import org.futurerobotics.jargon.model.MotorModel
import org.futurerobotics.jargon.model.TransmissionModel
import org.futurerobotics.jargon.printlnMe
import org.junit.jupiter.api.Test
import kotlin.math.pow

internal class DriveStateSpaceModelsTest {
    @Test
    fun `inspect nice numbers`() {
        val motor = MotorModel.fromMotorData(
            1 * volts,
            1.0 / 4, // 1/4 Nm/V
            1.0, //1 Ohm
            10.0, //.1 V/ (rad/s)
            0.0
        )
        val transmission = TransmissionModel.ideal(motor, 1.0)
        val mass = 5.0
        val model = FixedWheelDriveModel.mecanum(
            mass,
            0.5,
            transmission,
            0.05,
            .5,
            .5
        )
        model.run {
            listOf(
                motorVelFromBotVel,
                motorAccelFromVolts,
                voltsFromMotorVel,
                motorAccelFromMotorVel
            )
        }.forEach {
            println(it)
        }
        discretizeZeroOrderHold(
            DriveStateSpaceModels.decoupledMotorVelocityController(model, 0.0),
            1 / 20.0
        ).printlnMe()
        discretizeZeroOrderHold(DriveStateSpaceModels.poseVelocityController(model, model), 1 / 20.0).printlnMe()
    }

    @Test
    fun `inspect real holonomic`() {
        val motor = MotorModel.fromMotorData(
            12 * volts,
            260 * ozf * `in`,
            9.2 * A,
            435 * rev / mins,
            0.25 * A
        )
        val transmission = TransmissionModel.fromTorqueMultiplier(motor, 2.0, 50 * ozf * `in`, 0.9)
        val mass = 20 * lbm
        val model = FixedWheelDriveModel.mecanum(
            mass,
            mass / 12 * (18 * `in`).pow(2),
            transmission,
            2 * `in`,
            16 * `in`,
            14 * `in`
        )
        model.run {
            listOf(
                motorVelFromBotVel,
                motorAccelFromVolts,
                voltsFromMotorVel,
                motorAccelFromMotorVel
            )
        }.forEach {
            println(it)
        }
        discretizeZeroOrderHold(
            DriveStateSpaceModels.decoupledMotorVelocityController(model, 1.0),
            1 / 20.0
        ).printlnMe()
    }
}
