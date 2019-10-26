package org.futurerobotics.jargon.system

import java.util.concurrent.ExecutorService
import java.util.concurrent.Future
import java.util.concurrent.RejectedExecutionException

/**
 * Represents something with a life cycle, that can be run using [init], [run], and [stop].
 */
interface ExtendedRunnable {

    /**
     * Performs necessary initialization; the real work should _not_ be done here. This should only take
     * a short amount of time.
     */
    fun init()

    /**
     * Starts running the system. **This should only take a short amount of time, and is intended only
     * to start the running of another system.** For example, it can submit a task to a [ExecutorService] using
     * [SubmitTaskOnRun].
     */
    fun run()

    /** Attempts to stop the system immediately. */
    fun stop()
}

/**
 * A [ExtendedRunnable] that when run, submits a [task] to an [executor]; and attempts to cancel it upon
 * [stop][ExtendedRunnable.stop].
 *
 * When cancelled, the thread that runs the [task] is interrupted. Good people should periodically check if the current
 * thread is interrupted so that the task doesn't refuse to stop.
 */
class SubmitTaskOnRun(private val task: Runnable, private val executor: ExecutorService) : ExtendedRunnable {

    private lateinit var future: Future<*>
    override fun init() {
        check(!executor.isShutdown) { "Executor is shut down" }
    }

    override fun run() {
        try {
            future = executor.submit(task)
        } catch (e: RejectedExecutionException) {
            throw IllegalStateException("Cannot start task", e)
        }
    }

    override fun stop() {
        future.cancel(true)
    }
}
