package org.futurerobotics.jargon.profile

import kotlinx.coroutines.CompletableJob
import kotlinx.coroutines.Job

/**
 * Returns a [Job] that will complete when traversed pass the given [time].
 *
 * @see [TimeProfiledWithCallbacks.addCallback]
 */
fun TimeProfiledWithCallbacks<*, *>.addJob(time: Double): CompletableJob =
    Job().also {
        addCallback(time, Runnable { it.complete() })
    }

/**
 * Returns a [Job] that will complete when traversed pass the given [time].
 *
 * @see [TimeProfiledWithCallbacks.addCallback]
 */
fun <P : TimeProfiled<*>> TimeProfiledWithCallbacks<*, P>.addJob(time: ProfileTimeIndicator<P>): CompletableJob =
    Job().also {
        addCallback(time, Runnable { it.complete() })
    }

