package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.TAU
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.randomVectorDerivatives
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.*
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.junit.jupiter.api.Test
import java.io.File
import java.io.ObjectInputStream
import java.io.ObjectOutputStream
import java.nio.file.Paths
import kotlin.random.Random

internal class SerializationTest {

    @Test
    fun traj() {
        val path = List(5) {
            randomVectorDerivatives(Random(2384234), 10.0)
        }.zipWithNext { a, b ->
            QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(LinearlyInterpolatedHeading(0.0, TAU))
        }.let { multiplePath(it) }

        val constraints = MotionConstraintSet(
            MaxVelConstraint(5.0),
            MaxPathAngularVelConstraint(1.5),
            MaxCentripetalAccelConstraint(0.9),
            MaxTangentAccelConstraint(0.9),
            MaxTotalAccelConstraint(1.0),
            MaxAngularAccelConstraint(0.3)
        )

        val traj = generateTrajectory(path, constraints, MotionProfileGenParams())

        val file = File(Paths.get("./tmp/test/serialize/trajectory.tmp").toString())
        file.parentFile.mkdirs()
        println(file.absolutePath)

        ObjectOutputStream(file.outputStream()).use { it.writeObject(traj) }

        ObjectInputStream(file.inputStream()).use { it.readObject() as Trajectory }
        //just make sure it works
    }
}
