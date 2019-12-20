package org.futurerobotics.jargon.running

import kotlinx.coroutines.*
import org.junit.jupiter.api.Test
import strikt.api.expectThat
import strikt.assertions.all
import strikt.assertions.isEqualTo
import kotlin.random.Random
@UseExperimental(ExperimentalCoroutinesApi::class)
internal class TickerTest {

    @Test
    fun itTicks() = runBlocking {
        val ticker = ManualTicker()
        launch {
            repeat(5) {
                println("ticking $it")
                delay(10)
                ticker.tick()
            }
        }
        val listener = ticker.listener(1000)
        repeat(5) {
            listener.awaitNextTick()
            println("Awaited $it")
        }
    }

    @Test
    fun allTicks() = runBlocking<Unit> {
        val ticker = ManualTicker()
        val random = Random("Ticks work".hashCode())
        val times = 5

        class TestObj {
            var count = times * 2
            val job = launch {
                withTimeout(1200) {
                    val listener = ticker.listener()
                    listener.syncedLoop {
                        delay(random.nextLong(0, 100))
                        count--
                        false
                    }
                }
            }
        }

        val listeners = List(10) {
            TestObj()
        }
        delay(10)
        repeat(times) {
            ticker.tick()
        }
        ticker.tickTimes(times)
        listeners.forEach { it.job.join() }
        expectThat(listeners).all {
            get { count }.isEqualTo(0)
        }
    }
}
