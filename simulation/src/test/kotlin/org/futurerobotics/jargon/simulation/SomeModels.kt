package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.mechanics.DcMotorModel
import org.futurerobotics.jargon.mechanics.DriveModels
import org.futurerobotics.jargon.mechanics.TransmissionModel
import kotlin.math.pow

internal object SomeModels {
    private val motorModel = DcMotorModel.fromMotorData(
        12 * volts,
        260 * ozf * `in`,
        9.2 * A,
        435 * rev / min,
        0.25 * A
    )
    private val transmissionModel =
        TransmissionModel.fromTorqueLosses(motorModel, 2.0, 0.0, 0.9)
    val mecanum = run {
        val mass = 10.8 * lbs
        DriveModels.mecanumLike(
            mass,
            mass / 6 * (18 * `in`).pow(2),
            transmissionModel,
            2 * `in`,
            16 * `in`,
            14 * `in`
        )
    }
}
