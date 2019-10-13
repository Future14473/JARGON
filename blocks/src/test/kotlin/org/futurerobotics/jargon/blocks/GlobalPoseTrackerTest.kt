package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.math.*
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import kotlin.random.Random


internal class GlobalPoseTrackerTest {

    @Test
    fun `unit circle walk`() {
        //    x
        //    |
        // y<-+
        val (system, velocity, position) = getSystem(Pose2d(1.0, 0.0, 0.0))
        system.init()
        velocity.value = Pose2d(0.0, 1.0, 1.0)//1 radian per second CCW, 1 unit per second, forward
        expectThat(position) {
            val random = Random("unit circle walk".hashCode())
            var angle = 0.0
            repeat(200) {
                val angleStep = random.nextDouble(0.0, 2 * TAU)
                angle = angleNorm(angle + angleStep)
                system.loop(angleStep)
                get { value!! }.isEpsEqTo(Pose2d(Vector2d.polar(1, angle), angle))
            }
        }
    }

    @Test
    fun `random lines`() {
        val random = Random("random lines".hashCode())
        val (system, velocity, position) = getSystem(Pose2d.ZERO)
        var expectedPose = Pose2d.ZERO
        system.init()
        repeat(200) {
            val time = random.nextDouble()
            if (random.nextInt(3) == 0) {
                //rotate
                val rotate = random.nextDouble(-TAU, TAU)
                velocity.value = Pose2d(Vector2d.ZERO, rotate)
                expectedPose = expectedPose.angleRotated(rotate * time)
            } else {
                //move
                val move = Vector2d.random(random, 5.0)
                velocity.value = Pose2d(move, 0.0)
                expectedPose += Pose2d(move.rotated(expectedPose.heading) * time, 0.0)
            }
            system.loop(time)
            expectThat(position.value!!).describedAs("position").isEpsEqTo(expectedPose)
        }
    }

    private fun getSystem(initialPose: Pose2d): Triple<BlocksSystem, ExternalValue<Pose2d>, Monitor<Pose2d>> {
        val input = ExternalValue(Pose2d.ZERO)
        var monitor: Monitor<Pose2d>
        return Triple(buildBlocksSystem {
            val tracker = GlobalPoseTracker(initialPose)
            tracker.velocityIn connectFrom input
            monitor = tracker.monitor()
        }, input, monitor)
    }


}
