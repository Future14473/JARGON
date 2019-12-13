package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.trajectory.MotionConstraintSet
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.junit.jupiter.api.RepeatedTest
import strikt.api.expectThat
import strikt.assertions.isEqualTo
import kotlin.random.Random

class PointPathOk {
    private val random = Random("Point path is fine".hashCode())
    @RepeatedTest(5)
    fun itsOk() {
        val point = Pose2d(Vector2d.random(random, 3.0), random.nextDouble())
        val path = PointPath(point)
        val constraints = MotionConstraintSet()
        val trajectory = constraints.generateTrajectory(path, MotionProfileGenParams.DEFAULT)
        expectThat(trajectory) {
            get { duration }.isEqualTo(0.0)
            get { distance }.isEqualTo(0.0)
            get("First point") { stepper().stepTo(0.0) } and {
                get { value }.isEqualTo(point)
                get { deriv }.isEqualTo(Pose2d.ZERO)
                get { secondDeriv }.isEqualTo(Pose2d.ZERO)
            }
        }
    }
}
