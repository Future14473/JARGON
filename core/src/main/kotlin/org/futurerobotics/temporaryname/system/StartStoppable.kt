package org.futurerobotics.temporaryname.system

/**
 * Represents something that can be [start]ed and [stop]ped; meaning is abstract.
 */
interface StartStoppable {

    /**
     * Initializes; called at the start of a cycle
     */
    fun start()

    /**
     * Resets; Called on halt.
     */
    fun stop()
}