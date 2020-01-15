package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.math.convert.*
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.random.Random

internal class ModelsSanityTest {

    @Test
    fun `motor sanity check`() {
        val random = Random("sanity check".hashCode())
        repeat(100) {
            val r1 = random.nextDouble(-100.0, 100.0)
            val r2 = random.nextDouble(-100.0, 100.0)
            expectThat(idealUnitMotor.getVoltage(r1, r2)).isEpsEqTo(r1 + r2)
            expectThat(halfMotor.getVoltage(r1, r2)).isEpsEqTo(avg(r1, r2))
        }
    }

    @Test
    fun `differential works`() {
        val differential = FixedWheelDriveModel.differential(
            2.0, 3.0,
            TransmissionModel.fromTorqueMultiplier(idealUnitMotor, 2.0, 0.0, 0.5),
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
            get { voltsFromBotVel * forward }.isEpsEqTo(Vec(.4, .4))
            get { voltsFromBotVel * turn }.isEpsEqTo(Vec(-7.0 / 5 * 2, 7.0 / 5 * 2))
        }
    }

    @Test
    fun `inspect single wheel`() {
        val motor = idealUnitMotor
        val transmission = TransmissionModel.fromTorqueMultiplier(motor, 2.0, 0.0, 1.0)
        val loc = Vector2d(1, 1) / sqrt(2.0)
        val position = WheelPosition(loc, 1.0 zcross loc, 1.0)
        val wheelModel = FixedWheelModel(position, transmission)
        val model = FixedWheelDriveModel(1.0, 1.0, listOf(wheelModel))
        model.run {
            listOf(
                motorVelFromBotVel,
                motorAccelFromVolts,
                voltsFromMotorVel,
                motorAccelFromMotorVel
            )
        }.forEach {
            println(it.formatReadable())
        }
    }

    @Test
    fun `inspect holonomic`() {
        val motor = MotorModel.fromMotorData(
            12 * volts,
            260 * ozf * `in`,
            9.2 * A,
            435 * rev / mins,
            0.25 * A
        )
        val transmission = TransmissionModel.fromTorqueMultiplier(motor, 2.0, 50 * ozf * `in`, 0.9)
        val mass = 10.8 * lbs
        val model = FixedWheelDriveModel.mecanum(
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
            println(it.formatReadable())
        }
    }

    companion object {
        val idealUnitMotor = MotorModel.fromCoefficients(1.0, 1.0, 1.0, 0.0)

        val halfMotor = MotorModel.fromCoefficients(2.0, 1.0, 2.0, 0.0)
    }
}
