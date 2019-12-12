package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.math.ValueMotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.convert.*
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.randomVectorDerivatives
import org.futurerobotics.jargon.mechanics.FixedWheelDriveModel
import org.futurerobotics.jargon.mechanics.MotorModel
import org.futurerobotics.jargon.mechanics.TransmissionModel
import org.futurerobotics.jargon.pathing.TangentHeading
import org.futurerobotics.jargon.pathing.addHeading
import org.futurerobotics.jargon.pathing.multiplePath
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.MotionConstraintSet
import org.futurerobotics.jargon.pathing.trajectory.Trajectory
import org.futurerobotics.jargon.pathing.trajectory.generateTrajectory
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import kotlin.math.pow

internal object SomeModels {
    private val motorModel = MotorModel.fromMotorData(
        12 * volts,
        260 * ozf * `in`,
        9.2 * A,
        435 * rev / mins,
        0.25 * A
    )
    private val transmissionModel =
        TransmissionModel.fromTorqueMultiplier(motorModel, 2.0, 0.0, 0.9)
    val mecanum = run {
        val mass = 10.8 * lbs
        FixedWheelDriveModel.mecanumLike(
            mass,
            mass / 6 * (18 * `in`).pow(2),
            transmissionModel,
            2 * `in`,
            16 * `in`,
            14 * `in`
        )
    }
}

internal fun randomTrajectory(
    random: kotlin.random.Random,
    constraints: MotionConstraintSet
): Trajectory {
    val segs =
        (listOf(ValueMotionState(Vector2d.ZERO, Vector2d.polar(1.0, -74 * deg), Vector2d.ZERO)) +
                List(4) {
                    randomVectorDerivatives(random, 5.0)
                }).zipWithNext { a, b ->
            QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(TangentHeading(74 * deg))
        }
    val path = multiplePath(segs)
    return generateTrajectory(path, constraints, MotionProfileGenParams())
}
