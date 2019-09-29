package org.futurerobotics.jargon.system

/**
 * Represents something that can be [init]ed and [stop]ped, in any sense whatsoever
 */
interface InitStoppable {

    /** Initializes.*/
    fun init()

    /** Stops. */
    fun stop()
}