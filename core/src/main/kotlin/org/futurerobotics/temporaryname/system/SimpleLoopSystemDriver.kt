package org.futurerobotics.temporaryname.system

/**
 * Simple driver methods for a loop-based system, that is single threaded and not much more than a while loop.
 */
object SimpleLoopSystemDriver {

    /**
     * Runs the system until completion.
     */
    @JvmStatic
    fun runUntilCompletion(system: LoopBasedSystem) {
        try {
            while (true) {
                if (!system.tick() || Thread.interrupted()) break
            }
        } finally {
            system.stop()
        }
    }

    /**
     * Runs the system until completion or until it has looped [maxIterations] times.
     */
    @JvmStatic
    fun runMaxIterations(system: LoopBasedSystem, maxIterations: Int) {
        try {
            repeat(maxIterations) {
                if (!system.tick() || Thread.interrupted()) return
            }
        } finally {
            system.stop()
        }
    }
}