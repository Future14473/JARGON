package org.futurerobotics.jargon

import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.concurrent.withLock

/**
 * Runs the [block] in a separate thread, and interrupts it after the given [time] and [unit].
 */
fun interruptAfter(time: Long, unit: TimeUnit, block: () -> Unit) {
    val lock = ReentrantLock()
    val condition = lock.newCondition()

    var running = true
    val thread = thread {
        block()
        lock.withLock {
            running = false
            condition.signalAll()
        }
    }
    lock.withLock {
        if (running) condition.await(time, unit)
    }
    thread.interrupt()
}
