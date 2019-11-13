package org.futurerobotics.jargon

/**
 * Prints `this`, then returns `this`
 */
inline fun <T> T.printlnMe(): T = also { println(it) }
