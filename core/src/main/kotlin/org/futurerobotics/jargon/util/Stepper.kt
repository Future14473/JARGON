package org.futurerobotics.jargon.util

/**
 * A [Stepper] is similar to an [Iterator], except that on each iteration (called a "step"), a double value
 * must be specified as to where to "step" to.
 *
 * _This is distinguished from a mapping function in that it is used to apply a heuristic that values between
 * steps are close together in some way_, for example when traversing a path.
 *
 * This can be used, for example, to avoid doing binary search on every iteration when traversing over values, while
 * also not wasting memory on intermediary lists.
 */
interface Stepper<out R> {

    /**
     * Steps to the given [step], and returns the value.
     *
     * A invalid value of [step] produces undefined behavior.
     */
    fun stepTo(step: Double): R
}

/** Returns a stepper that uses the supplied [step] function for stepping. */
@Suppress("FunctionName")
inline fun <R> Stepper(crossinline step: (Double) -> R): Stepper<R> = object : Stepper<R> {
    override fun stepTo(step: Double): R = step(step)
}

/** Represents something that can be stepped with a [Stepper]. */
interface Steppable<out R> {

    /** Gets a stepper for this [Steppable]. */
    fun stepper(): Stepper<R>
}

/** Returns a [Steppable] that uses the given [stepper] function to provide a stepper. */
@Suppress("FunctionName")
inline fun <R> Steppable(crossinline stepper: () -> Stepper<R>): Steppable<R> = object : Steppable<R> {
    override fun stepper(): Stepper<R> = stepper()
}

/** Returns a list corresponding to stepping through all the values in the specified [list]. */
fun <R> Stepper<R>.stepToAll(list: Iterable<Double>): List<R> = list.map { stepTo(it) }

/**
 * Returns a list corresponding to stepping through all the values in the specified [list].
 * This assumes that all values in [list] can be stepped through.
 */
fun <R> Steppable<R>.stepToAll(list: Iterable<Double>): List<R> = stepper().stepToAll(list)
