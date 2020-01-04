package org.futurerobotics.jargon.callback

import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.profile.*
import org.futurerobotics.jargon.util.stepToAll
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import java.util.concurrent.CompletionStage

internal class TimeProfiledWithCallbacksTest {

    @Test
    fun itCallsBack() {
        val profile = object : TimeDistanceProfiled<Any> {
            override val duration: Double
                get() = 1.0

            override fun atTime(time: Double): Any = Unit

            override val distance: Double
                get() = 1.0

            override fun atDistance(distance: Double): Any = Unit

            override fun timeAtDistance(distance: Double): Double = distance
        }
        val callbacks = listOf(
            AtBegin(),
            AtTime(0.05),
            AtDistance(0.2),
            AtDistance(0.4),
            AtEnd()
        )
        val withCallbacks = profile.withCallbacks()
        var index = 0
        var reqIndex = 0
        callbacks.map {
            withCallbacks.addCallback(it)
        }.onEach {
            val required = reqIndex++
            (it as CompletionStage<*>).toCompletableFuture().thenRun {
                expectThat(required == index++)
            }
        }
        withCallbacks
            .stepper()
            .stepToAll(
                DoubleProgression.fromNumSegments(
                    0.0,
                    1.0,
                    100
                )
            )
    }
}
