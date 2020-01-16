package org.futurerobotics.jargon

/**
 * Prints `this`, then returns `this`
 */
fun <T> T.printlnMe(): T = also { println(it) }
