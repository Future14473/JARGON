package org.futurerobotics.jargon

import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * Runs the [block] in this thread, and interrupts it after the given [time] and [unit],
 * then tries to clear the interrupt flag.
 */
inline fun interruptAfter(time: Long, unit: TimeUnit, block: () -> Unit) {

    val runnerThread = Thread.currentThread()
    val interrupter = thread {
        try {
            Thread.sleep(unit.toMillis(time))
            runnerThread.interrupt()
        } catch (e: InterruptedException) {

        }
    }
    block()
    interrupter.interrupt()
    Thread.interrupted()
}
