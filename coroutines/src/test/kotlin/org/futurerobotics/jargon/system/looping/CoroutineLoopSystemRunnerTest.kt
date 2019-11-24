package org.futurerobotics.jargon.system.looping

import kotlinx.coroutines.runBlocking
import org.junit.jupiter.api.RepeatedTest
import org.junit.jupiter.api.Test
import strikt.api.expect
import strikt.assertions.isEqualTo
import strikt.assertions.isIn
import kotlin.math.roundToLong
import kotlin.system.measureNanoTime

private class RunTimes(var times: Int) : LoopSystem {
    var number = 0

    var finalNumber = 0
    override fun start() {
        number = 0
    }

    override fun loop(loopTimeInNanos: Long): Boolean {
        number++
        return number >= times
    }

    override fun stop() {
        finalNumber = number
    }
}

internal class SimpleLoopSystemDriverTest {
    @Test
    fun `it stops`() {
        expect {
            repeat(10) {
                val system = RunTimes(it + 1)
                CoroutineLoopSystemRunner(system, LoopAsFastAsPossible()).run()
                that(system.finalNumber).isEqualTo(it + 1)
            }
        }
    }

    private fun warmUp() = runBlocking {
        CoroutineLoopSystemRunner(RunTimes(5000), LoopAsFastAsPossible()).run()
    }

    @RepeatedTest(4)
    fun `regulation works`() { //< a second, mostly sleeping
        val hertz = 100.0
        val regulator = LoopWithMaxSpeed((1e9 / hertz).roundToLong())
        val system = RunTimes(1)//only regulated on 2nd cycle onward.
        warmUp()
        for (times in 10..12) {
            system.times = times + 1 //regulated 1 less time.
            val runner = CoroutineLoopSystemRunner(system, regulator)
            expect {
                val time = measureNanoTime {
                    runner.run()
                } / 1e9
                that(time).isIn((times / hertz).let { 0.95 * it..1.8 * it })//it might be slow
            }
        }
    }
}
