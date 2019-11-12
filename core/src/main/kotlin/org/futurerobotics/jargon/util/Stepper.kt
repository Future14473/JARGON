package org.futurerobotics.jargon.util

/**
 * A [Stepper] is similar to an [Iterator], except that on each iteration (called a "step"), an input value of type [T]
 * must be specified as to where to "step"/iterate to.
 *
 * _This is distinguished from a mapping function in that it is used to apply a heuristic that values between
 * steps are close together in some way_.
 *
 * This can be used, for example, to avoid doing binary search on every iteration when traversing over values, while
 * also not wasting memory on intermediary lists.
 */
interface Stepper<in T, out R> {

    /**
     * Steps to the given [step], and returns the value.
     * A invalid value of [step] produces undefined behavior.
     */
    fun stepTo(step: T): R
}

/** Returns a stepper that uses the supplied [step] function for stepping. */
@Suppress("FunctionName")
inline fun <T, R> Stepper(crossinline step: (T) -> R): Stepper<T, R> = object : Stepper<T, R> {
    override fun stepTo(step: T): R = step(step)
}

/** Represents something that can be stepped with a [Stepper]. */
interface Steppable<in T, out R> {

    /** Gets a stepper for this [Steppable]. */
    fun stepper(): Stepper<T, R>
}

/** Returns a [Steppable] that uses the given [stepper] function to provide a stepper. */
@Suppress("FunctionName")
inline fun <T, R> Steppable(crossinline stepper: () -> Stepper<T, R>): Steppable<T, R> = object : Steppable<T, R> {
    override fun stepper(): Stepper<T, R> = stepper()
}

/** Returns a list corresponding to stepping through all the values in the specified [list]. */
fun <T, R> Stepper<T, R>.stepToAll(list: Iterable<T>): List<R> = list.map { stepTo(it) }

/**
 * Returns a list corresponding to stepping through all the values in the specified [list].
 * This assumes that all values in [list] can be stepped through.
 */
fun <T, R> Steppable<T, R>.stepToAll(list: Iterable<T>): List<R> = stepper().stepToAll(list)
