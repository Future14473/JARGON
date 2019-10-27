package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.TAU
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.math.randomVectorDerivatives
import org.futurerobotics.jargon.pathing.reparam.reparamByIntegration
import org.futurerobotics.jargon.pathing.trajectory.*
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
            QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(LinearInterpolatedHeading(0.0, TAU))
        }.let {
            it + Line(Vector2d(3.0, 2.0), Vector2d(2.1, 3.4)).addHeading(TangentHeading) +
                    PointTurn(Vector2d(4, 3), 30.0, 20.0)
        }.let { MultiplePath(it, false) }

        val constraints = MotionConstraintSet(
            MaxVelConstraint(5.0),
            MaxPathAngularVelConstraint(1.5),
            MaxCentripetalAccelConstraint(0.9),
            MaxTangentAccelConstraint(0.9),
            MaxTotalAccelConstraint(1.0),
            MaxAngularAccelConstraint(0.3)
        )

        val traj = constraints.generateTrajectory(path)

        val file = File(Paths.get("./tmp/test/serialize/trajectory.tmp").toString())
        file.parentFile.mkdirs()
        println(file.absolutePath)

        ObjectOutputStream(file.outputStream()).use { it.writeObject(traj) }

        val input = ObjectInputStream(file.inputStream()).use { it.readObject() as Trajectory }
        //just make sure it works...
    }
}
