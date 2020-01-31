package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.math.zcross
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import kotlin.math.pow
import kotlin.math.sqrt

internal class ModelsSanityTest {

    @Test
    fun `differential works`() {
        val differential = DriveModel.differential(
            2.0, 3.0,
            OldTransmissionModel.fromTorqueMultiplier(idealUnitMotor, 2.0, 0.0, 0.5),
            5.0, 7.0
        )
        expectThat(differential) {
            val forward = Pose2d(1.0, 0.0, 0.0).toVec()
            val turn = Pose2d(0.0, 0.0, 1.0).toVec()
            //prime numbers!
            //mass 2.0
            //moi: 3.0
            //wheel radius: 5.0
            //placement radius: 7.0
            //gear ratio: 2
            //transmission loss: 1/2
            //motor: unit
            //at output: 1, At transmission: 1/5. At motor: 2/5
            get { voltsFromBotVel * forward }.isEpsEqTo(vecOf(.4, .4))
            get { voltsFromBotVel * turn }.isEpsEqTo(vecOf(-7.0 / 5 * 2, 7.0 / 5 * 2))
        }
    }

    @Test
    fun `inspect single wheel`() {
        val motor = idealUnitMotor
        val transmission = OldTransmissionModel.fromTorqueMultiplier(motor, 2.0, 0.0, 1.0)
        val loc = Vector2d(1, 1) / sqrt(2.0)
        val position = DriveWheelPosition(loc, 1.0 zcross loc, 1.0)
        val wheelModel = FixedWheelModel(position, transmission)
        val model = DriveModel(listOf(wheelModel), 1.0, 1.0)
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
    }

    @Test
    fun `inspect holonomic`() {
        val motor = DcMotorModel.fromMotorData(
            12 * volts,
            260 * ozf * `in`,
            9.2 * A,
            435 * rev / mins,
            0.25 * A
        )
        val transmission = OldTransmissionModel.fromTorqueMultiplier(motor, 2.0, 50 * ozf * `in`, 0.9)
        val mass = 10.8 * lbm
        val model = DriveModel.mecanum(
            mass,
            mass / 4 * (18 * `in`).pow(2),
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
    }

    companion object {
        val idealUnitMotor = DcMotorModel.fromCoefficients(1.0, 1.0, 1.0, 0.0)

        val halfMotor = DcMotorModel.fromCoefficients(2.0, 1.0, 2.0, 0.0)
    }
}
