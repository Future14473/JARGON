package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.mechanics.DcMotorModel
import org.futurerobotics.jargon.mechanics.DriveModels
import org.futurerobotics.jargon.mechanics.TransmissionModel
import org.junit.jupiter.api.Test
import strikt.api.expectCatching
import strikt.assertions.failed
import strikt.assertions.succeeded
import kotlin.math.pow

private val motorModel = DcMotorModel.fromMotorData(
    12 * volts,
    260 * ozf * `in`,
    9.2 * A,
    435 * rev / min,
    0.25 * A
)
private val transmissionModel = TransmissionModel.fromTorqueLosses(motorModel, 2.0, 0.0, 0.9)
private const val mass = 10.8 * lbs
private val driveModel = DriveModels.mecanumLike(
    mass,
    mass / 6 * (18 * `in`).pow(2),
    transmissionModel,
    2 * `in`,
    16 * `in`,
    14 * `in`
)

internal class DecoupledTest {
    @Test
    fun `will it converge`() {
        var ssModel = LinearDriveModels.motorVelocityController(driveModel)
        //not controllable
        expectCatching {
            continuousLQR(ssModel, QRCost(idenMat(ssModel.stateSize), idenMat(ssModel.inputSize)))
        }.failed()
        ssModel = LinearDriveModels.decoupledMotorVelocityController(driveModel, 0.5)
        expectCatching {
            continuousLQR(ssModel, QRCost(idenMat(ssModel.stateSize), idenMat(ssModel.inputSize)))
        }.succeeded()
    }
}
