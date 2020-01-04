package org.futurerobotics.jargon.running

/**
 * Something that can be [init]ed nad [stop]ped.
 *
 * Meaning can be anything whatsoever, depending on context.
 */
interface InitStoppable {

    /**
     * Initializes.
     */
    fun init()

    /**
     * Stops.
     */
    fun stop()
}
