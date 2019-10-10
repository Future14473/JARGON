package org.futurerobotics.jargon.control


/**
 * An interface that provides [BlockInput] and [BlockOutput]s for information that taps directly into
 * the life of the system.
 */
interface SystemValues {
    /**
     * Shutdown; when inputted 'true', will tell the system to stop running next cycle.
     */
    val shutdown: BlockInput<Boolean?>

    /**
     * Output for the number of the current loop, starting with 0 when the system first starts.
     */
    val loopNumber: BlockOutput<Int>

    /**
     * Output for the time it took for the last loop to run; effectively the best estimate of how fast the
     * system is running.
     */
    val loopTime: BlockOutput<Double>
}