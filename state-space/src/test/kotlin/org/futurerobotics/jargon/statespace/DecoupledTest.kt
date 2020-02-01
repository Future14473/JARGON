package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.model.DcMotorModel
import org.futurerobotics.jargon.model.DriveModel
import org.futurerobotics.jargon.model.DriveModels
import org.futurerobotics.jargon.model.ModifiedPowerModel
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.assertions.failed
import strikt.assertions.succeeded
import kotlin.math.pow

private val motorModel = DcMotorModel.fromMotorData(
    12 * volts,
    260 * ozf * `in`,
    9.2 * A,
    435 * rev / mins,
    0.25 * A
)
private val powerModel = ModifiedPowerModel.of(motorModel, 0.9)
private const val mass = 10.8 * lbm
private val driveModel =
    DriveModel(
        DriveModels.mecanum(
            2 * `in`,
            2.0,
            16 * `in`,
            14 * `in`
        ),
        powerModel,
        mass,
        mass / 6 * (18 * `in`).pow(2)
    )

internal class DecoupledTest {
    @Test
    fun `will it converge`() {
        var mats = DriveStateSpaceModels.motorVelocityController(driveModel)
        //not controllable
        expectCatching {
            continuousLQR(mats.A, mats.B, idenMat(mats.numStates), idenMat(mats.numInputs))
        }.failed()
        mats = DriveStateSpaceModels.decoupledMotorVelocityController(driveModel, 0.5)
        expectCatching {
            continuousLQR(mats.A, mats.B, idenMat(mats.numStates), idenMat(mats.numInputs))
        }.succeeded()
    }
}
