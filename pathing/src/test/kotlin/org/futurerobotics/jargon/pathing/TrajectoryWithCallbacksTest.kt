package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.pathing.trajectory.MaxVelConstraint
import org.futurerobotics.jargon.pathing.trajectory.MotionConstraintSet
import org.futurerobotics.jargon.profile.MotionProfileGenParams
import org.futurerobotics.jargon.running.CompletionCallback
import org.futurerobotics.jargon.util.stepToAll
import org.junit.jupiter.api.Test
import strikt.api.expect
import strikt.assertions.isEqualTo

internal class TrajectoryWithCallbacksTest {

    @Test
    fun itCallsBack() {
        val constrants = MotionConstraintSet(MaxVelConstraint(1.0))
        val path = Line(Vector2d.ZERO, Vector2d(1.0, 2.0)).addHeading(TangentHeading)
        val trajectory = constrants.generateTrajectory(path, MotionProfileGenParams())
        var count = 0

        class IndexCallback : CompletionCallback {
            var index = -1
            override fun complete() {
                index = count++
            }

            override fun cancel() {
            }
        }

        val futures = mutableListOf<IndexCallback>()
        fun new(): CompletionCallback = IndexCallback().also { futures += it }

        val callbacks = listOf(
            TrajectoryStartedCallback(new()),
            TimeCallback(0.05, new()),
            DistanceCallback(1.0, new()),
            DistanceCallback(2.0, new()),
            TrajectoryCompleteCallback(new())
        )
        TrajectoryWithCallbacks(trajectory, callbacks.shuffled())
            .stepper()
            .stepToAll(
                DoubleProgression.fromNumSegments(
                    0.0,
                    TrajectoryWithCallbacks(trajectory, callbacks.shuffled()).duration,
                    100
                )
            )
        expect {
            futures.forEachIndexed { index, it ->
                that(it).get("index") { it.index }.isEqualTo(index)
            }
        }
    }
}
