package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.avg
import org.futurerobotics.jargon.math.isEpsEqTo
import org.junit.jupiter.api.Test
import strikt.api.expect
import strikt.api.expectThat
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
            get { motorVoltsPerOutputTorque }.isEpsEqTo(1.0)
        }
        that(halfLossTransmission) {
            get { motorVelPerOutputVel }.isEpsEqTo(1.0)
            get { motorTorquePerOutputTorque }.isEpsEqTo(2.0)
            get { motorVoltsPerOutputTorque }.isEpsEqTo(2.0)
        }
        that(negativeGearedTransmission) {
            get { motorVelPerOutputVel }.isEpsEqTo(-2.0)
            get { motorTorquePerOutputTorque }.isEpsEqTo(-0.5)
            get { motorVoltsPerOutputTorque }.isEpsEqTo(-0.5)
        }
    }

    @Test
    fun `differential works`() {
        val differential = DriveModels.differential(
            2.0, 3.0,
            TransmissionModel.fromTorqueLosses(idealUnitMotor, 2.0, 0.0, 0.5),
            5.0, 7.0
        )
        expectThat(differential) {
            val forward = Pose2d(1.0, 0.0, 0.0).toVector()
            val turn = Pose2d(0.0, 0.0, 1.0).toVector()
            //prime numbers!
            //mass 2.0
            //moi: 3.0
            //wheel radius: 5.0
            //placement radius: 7.0
            //gear ratio: 2
            //transmission loss: 1/2
            //motor: unit
            get { wheelVelFromBotVel * forward } isEpsEqTo vec[1, 1] //at output: [1,1],
            get { wheelVelFromBotVel * turn } isEpsEqTo vec[-7, 7] //since radius = 7
            //at output: 1, At transmission: 1/5. At motor: 2/5
            get { voltsFromBotVel * forward } isEpsEqTo vec[.4, .4]
            get { voltsFromBotVel * turn } isEpsEqTo vec[-7.0 / 5 * 2, 7.0 / 5 * 2]
            //2N / 2 motors *(wheel radius), 5, transmission loss/2, gear ratio * 2
            get { voltsFromBotAccel * forward } isEpsEqTo vec[5.0, 5.0]
            //3 Nm/7m/2 *5/4
            get { voltsFromBotAccel * turn } isEpsEqTo vec[-3.0 / 7 / 2 * 5, 3.0 / 7 / 2 * 5]
        }

    }

    companion object {
        val idealUnitMotor = DcMotorModel.fromCoefficients(1.0, 1.0, 1.0, 0.0)

        val halfMotor = DcMotorModel.fromCoefficients(2.0, 1.0, 2.0, 0.0)

        val idealUnitTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, 1.0)

        val halfLossTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, 1.0, 0.0, 0.5)

        val negativeGearedTransmission = TransmissionModel.fromTorqueLosses(idealUnitMotor, -2.0)


    }
}