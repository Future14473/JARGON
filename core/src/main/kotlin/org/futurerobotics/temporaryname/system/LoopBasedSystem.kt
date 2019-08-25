package org.futurerobotics.temporaryname.system

/**
 * Represents a simple loop-based system, where a loop is run repeatedly until it ends.
 */
interface LoopBasedSystem {
    /**
     * Called be called before the system starts.
     */
    fun init()

    /**
     * Runs one cycle of the loop and returns false if loop is completed/should be halted.
     * @return false if this system has completed, true to continue.
     */
    fun tick(): Boolean

    /**
     * Called after the system ended or if interrupted.
     */
    fun stop()
}