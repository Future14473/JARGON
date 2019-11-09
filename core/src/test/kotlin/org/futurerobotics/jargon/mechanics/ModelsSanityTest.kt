package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.avg
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.math.isEpsEqTo
import org.junit.jupiter.api.Test
import strikt.api.expect
import strikt.api.expectThat
import kotlin.math.pow
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
    fun `transmission sanity check`() = expect {
        that(idealUnitTransmission) {
            get { motorVelPerOutputVel }.isEpsEqTo(1.0)
            get { motorTorquePerOutputTorque }.isEpsEqTo(1.0)
            get { voltsPerOutputTorque }.isEpsEqTo(1.0)
        }
        that(halfLossTransmission) {
            get { motorVelPerOutputVel }.isEpsEqTo(1.0)
            get { motorTorquePerOutputTorque }.isEpsEqTo(2.0)
            get { voltsPerOutputTorque }.isEpsEqTo(2.0)
        }
        that(negativeGearedTransmission) {
            get { motorVelPerOutputVel }.isEpsEqTo(-2.0)
            get { motorTorquePerOutputTorque }.isEpsEqTo(-0.5)
            get { voltsPerOutputTorque }.isEpsEqTo(-0.5)
        }
    }

    @Test
    fun `differential works`() {
        val differential = NominalDriveModels.differential(
            2.0, 3.0,
            TransmissionModel.fromTorqueLosses(idealUnitMotor, 2.0, 0.0, 0.5),
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
    fun inspect() {
        val motorModel = MotorModel.fromMotorData(
            12 * volts,
            260 * ozf * `in`,
            9.2 * A,
            435 * rev / min,
            0.25 * A
        )
        val transmissionModel =
            TransmissionModel.fromTorqueLosses(motorModel, 2.0, 0.0, 0.9)
        val mass = 10.8 * lbs
        val mecanum = NominalDriveModels.mecanumLike(
            mass,
            mass / 6 * (18 * `in`).pow(2),
            transmissionModel,
            2 * `in`,
            16 * `in`,
            14 * `in`,
            Vector2d(-5 * `in`, 0.0)
        )
        println(mecanum.motorVelFromBotVel.formatReadable())
    }

    companion object {
        val idealUnitMotor = MotorModel.fromCoefficients(1.0, 1.0, 1.0, 0.0)

        val halfMotor = MotorModel.fromCoefficients(2.0, 1.0, 2.0, 0.0)

        val idealUnitTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, 1.0)

        val halfLossTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, 1.0, 0.0, 0.5)

        val negativeGearedTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, -2.0)
    }
}
