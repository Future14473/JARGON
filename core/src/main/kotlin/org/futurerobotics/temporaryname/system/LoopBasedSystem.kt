package org.futurerobotics.temporaryname.system

/**
 * Represents a simple loop-based system, where a loop is run repeatedly until it ends.
 */
interface LoopBasedSystem : StartStoppable {

    /**
     * Runs one cycle of the loop.
     *
     * returns false if loop is completed/should be halted, true to continue.
     */
    fun tick(): Boolean
}