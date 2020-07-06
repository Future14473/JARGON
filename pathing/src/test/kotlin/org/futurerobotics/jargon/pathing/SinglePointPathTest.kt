package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.trajectory.MotionConstraintSet
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.junit.jupiter.api.RepeatedTest
import strikt.api.expectThat
import strikt.assertions.isEqualTo
import kotlin.random.Random

class SinglePointPathTest {
    private val random = Random("Point path is fine".hashCode())

    @RepeatedTest(5)
    fun itsOk() {
        val point = Pose2d(Vector2d(random.nextDouble(-3.0, 3.0), random.nextDouble(-3.0, 3.0)), random.nextDouble())
        val path = SinglePointPath(point)
        val constraints = MotionConstraintSet()
        val trajectory = constraints.generateTrajectory(path, MotionProfileGenParams.DEFAULT)
        expectThat(trajectory) {
            get { duration }.isEqualTo(0.0)
            get { distance }.isEqualTo(0.0)
            get("First point") { timeStepper().stepTo(0.0) } and {
                get { value }.isEqualTo(point)
                get { deriv }.isEqualTo(Pose2d.ZERO)
                get { secondDeriv }.isEqualTo(Pose2d.ZERO)
            }
        }
    }
}