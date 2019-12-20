package org.futurerobotics.jargon.running

import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.futurerobotics.jargon.util.value
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicReference

/**
 * A ticker is a utility to help multiple (possibly concurrent) systems stay in sync to the same "tick".
 * Suspend and blocking functions are supported.
 *
 * This is done by getting a [listener] that waits for "tick"s to happen. Listeners will also keep track
 * of up to a configurable number of "missed" ticks: if calls to wait for ticks run slower than the ticks,
 * it will not wait for the following tick but immediately continue.
 *
 * All methods are be thread-safe.
 *
 * @see ManualTicker
 */
interface Ticker {

    /** Creates a new [TickerListener]. See [Ticker] doc. */
    fun listener(maximumTicksBehind: Int = Int.MAX_VALUE): TickerListener
}

/**
 * Listens for ticks. See [Ticker] doc.
 */
interface TickerListener {

    /**
     * The maximum number of ticks this [TickerListener] is allowed to trail behind.
     */
    var maximumTicksBehind: Int

    /**
     * Resets so that there is no number of ticks this is behind.
     */
    fun reset()

    /**
     * Awaits the next tick, or returns immediately if the next tick has already passed.
     *
     * This will throw CancellationException if coroutine is cancelled.
     */
    suspend fun awaitNextTick()

    /**
     * Awaits the next given number of [ticks], or returns immediately if that many ticks has already passed.
     *
     * This will throw CancellationException if coroutine is cancelled.
     */
    suspend fun awaitNextTicks(ticks: Int)

    /**
     * Awaits the next tick, or returns immediately if the next tick has already passed.
     *
     * This will throw InterruptedException if thread is interrupted.
     */
    @Throws(InterruptedException::class)
    fun awaitNextTickBlocking()

    /**
     * Awaits the next given number of [ticks], blocking, or returns immediately if that many ticks has already passed.
     *
     * This will throw InterruptedException if thread is interrupted.
     */
    @Throws(InterruptedException::class)
    fun awaitNextTicksBlocking(ticks: Int)

    /**
     * Returns true if the next tick has passed.
     */
    fun isNextTickPassed(): Boolean

    /**
     * Returns true if a given number of ticks has passed.
     */
    fun isNextTicksPassed(ticks: Int): Boolean
}

/**
 * Base implementation of [Ticker], where ticks can be made via the [tick] or [tickTimes] function.
 */
abstract class BaseTicker : Ticker {

    private val tick = AtomicReference<Tick>(
        Tick(0, CompletableDeferred())
    )

    /** Ticks once. */
    protected open fun tick() {
        tickTimes(1)
    }

    /** Ticks a number of [times]. */
    protected open fun tickTimes(times: Int) {
        val deferred = CompletableDeferred<Nothing?>()
        tick.getAndUpdate {
            //replace with next number, atomically.
            Tick(it.tickNum + times, deferred)
        }.deferred.complete(null) //complete previous.
    }

    final override fun listener(maximumTicksBehind: Int): TickerListener = TickerListenerImpl(maximumTicksBehind)

    private class Tick(val tickNum: Int, val deferred: CompletableDeferred<Nothing?>) {

        fun isTickPassed(num: Int): Boolean = num - tickNum < 0
    }

    private inner class TickerListenerImpl(override var maximumTicksBehind: Int) :
        TickerListener {

        private val curPassedTick = AtomicInteger(tick.value.tickNum - 1)

        override fun reset() {
            curPassedTick.value = tick.value.tickNum - 1
        }

        override suspend fun awaitNextTick() {
            awaitNextTicks(1)
        }

        override suspend fun awaitNextTicks(ticks: Int) {
            val waitFor = getToWaitFor(ticks)
            var tick = this@BaseTicker.tick.value
            if (tick.isTickPassed(waitFor)) {
                yield()
            } else {
                do {
                    tick.deferred.await()
                    tick = this@BaseTicker.tick.value
                } while (!tick.isTickPassed(waitFor))
            }
        }

        private fun getToWaitFor(ticks: Int): Int =
            if (maximumTicksBehind == Int.MAX_VALUE) curPassedTick.incrementAndGet()
            else {
                val minNext = tick.value.tickNum - maximumTicksBehind
                curPassedTick.updateAndGet {
                    val next = (it + ticks)
                    if (next - minNext < 0) minNext else next
                }
            }

        override fun awaitNextTickBlocking() {
            awaitNextTicksBlocking(1)
        }

        override fun awaitNextTicksBlocking(ticks: Int) {
            val waitFor = getToWaitFor(ticks)
            var tick = this@BaseTicker.tick.value
            if (tick.isTickPassed(waitFor)) {
                if (Thread.interrupted()) throw InterruptedException()
            } else runBlocking {
                do {
                    tick.deferred.await()
                    tick = this@BaseTicker.tick.value
                } while (!tick.isTickPassed(waitFor))
            }
        }

        override fun isNextTickPassed(): Boolean = tick.value.isTickPassed(curPassedTick.get())

        override fun isNextTicksPassed(ticks: Int): Boolean = tick.value.isTickPassed(curPassedTick.get() + ticks - 1)
    }
}

/**
 * A ticker in which [tick]s can be done manually.
 */
class ManualTicker : BaseTicker() {

    public override fun tick(): Unit = super.tick()

    public override fun tickTimes(times: Int): Unit = super.tickTimes(times)
}

/**
 * A ticker that uses a [SuspendFrequencyRegulator] to run ticks.
 *
 * Some coroutine needs to [runSuspend] this in order to work.
 *
 * @see TickerListeningRegulator
 */
class FrequencyRegulatedTicker(private val regulator: SuspendFrequencyRegulator) : BaseTicker(),
                                                                                   SuspendRunnable {

    private val system = object : LoopSystem {
        override fun loop(loopTimeInNanos: Long): Boolean {
            tick()
            return false
        }
    }

    override suspend fun runSuspend() {
        SuspendLoopSystemRunner(system, regulator).runSuspend()
    }
}

/**
 * A [SuspendFrequencyRegulator] that uses a [TickerListener] to regulate loops. It will simply use
 * the given [clock] to evaluate time.
 *
 * @see FrequencyRegulatedTicker
 */
class TickerListeningRegulator
@JvmOverloads constructor(
    private val listener: TickerListener, private val clock: Clock = Clock.DEFAULT
) : SuspendFrequencyRegulator {

    private var lastNanos = clock.nanoTime()
    override fun start() {
        lastNanos = clock.nanoTime()
        listener.reset()
    }

    override suspend fun syncSuspend() {
        listener.awaitNextTick()
    }

    override fun sync() {
        listener.awaitNextTickBlocking()
    }

    override fun getElapsedNanos(): Long {
        val nanoTime = clock.nanoTime()
        return (nanoTime - lastNanos)
            .also { lastNanos = nanoTime }
    }

    override fun stop() {
    }
}

/** Convenience extension function for [TickerListeningRegulator]. */
@JvmOverloads
fun TickerListener.asFrequencyRegulator(clock: Clock = Clock.DEFAULT): TickerListeningRegulator =
    TickerListeningRegulator(this, clock)

/**
 * Runs an loop with the [block] that syncs with this [TickerListener] before every loop,
 * and breaks when the [block]'s return is `true`, or throws if coroutine is cancelled. This uses suspend functions.
 */
suspend inline fun TickerListener.syncedLoop(block: () -> Boolean) {
    while (true) {
        awaitNextTick() //throws if cancelled here
        if (block()) break
    }
}

/**
 * Runs an loop with the [block] that syncs with this [TickerListener] every loop,
 * and breaks when the [block]'s return is `true`, or throws if the thread is interrupted.
 */
@Throws(InterruptedException::class)
inline fun TickerListener.syncedLoopBlocking(block: () -> Boolean) {
    while (true) {
        awaitNextTickBlocking() //throws if interrupted here
        if (block()) break
    }
}
