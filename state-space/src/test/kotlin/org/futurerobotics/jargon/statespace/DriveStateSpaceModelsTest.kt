package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.model.DcMotorModel
import org.futurerobotics.jargon.model.DriveModel
import org.futurerobotics.jargon.model.DriveModels
import org.futurerobotics.jargon.printlnMe
import org.junit.jupiter.api.Test

internal class DriveStateSpaceModelsTest {
    @Test
    fun `inspect nice numbers`() {
        val motor = DcMotorModel.fromMotorData(
            1 * volts,
            1.0 / 4, // 1/4 Nm/V
            1.0, //1 Ohm
            10.0, //.1 V/ (rad/s)
            0.0
        )
        val mass = 5.0
        val model = DriveModels.mecanum(
            0.05,
            0.5,
            .5,
            .5
        ).let {
            DriveModel(it, motor, mass, mass)
        }
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
        val motor = DcMotorModel.fromMotorData(
            12 * volts,
            260 * ozf * `in`,
            9.2 * A,
            435 * rev / mins,
            0.25 * A
        )
        val mass = 5.0
        val model = DriveModels.mecanum(
            0.05,
            0.5,
            .5,
            .5
        ).let {
            DriveModel(it, motor, mass, mass)
        }
        discretizeZeroOrderHold(
            DriveStateSpaceModels.decoupledMotorVelocityController(model, 1.0),
            1 / 20.0
        ).printlnMe()
    }
}
