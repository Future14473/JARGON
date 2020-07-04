package org.futurerobotics.jargon.util

/**
 * A [Stepper] is similar to an [Iterator], except that on each "step", a double value
 * is specified for where to "step" to.
 *
 * This is distinguished from a simple function in that it is used to apply a heuristic/optimization that the values
 * given between steps are close together. for example when traversing interpolated segments, which are traversed
 * from one end to the other.
 *
 * This can be used to avoid doing binary search on every iteration while also not wasting memory on intermediary lists.
 */
interface Stepper<out T> {

    /**
     * Steps to the given [step], and returns the value.
     *
     * A invalid value of [step] produces undefined behavior.
     */
    fun stepTo(step: Double): T
}

/** Returns a stepper that uses the supplied [step] function for stepping. */
@Suppress("FunctionName")
inline fun <T> Stepper(crossinline step: (Double) -> T): Stepper<T> = object : Stepper<T> {
    override fun stepTo(step: Double): T = step(step)
}

/** Represents something that can be stepped with a [Stepper]. */
interface Steppable<out T> {

    /** Gets a stepper for this [Steppable]. */
    fun stepper(): Stepper<T>
}

/** Returns a [Steppable] that uses the given [stepper] function to provide a stepper. */
@Suppress("FunctionName")
inline fun <T> Steppable(crossinline stepper: () -> Stepper<T>): Steppable<T> = object : Steppable<T> {
    override fun stepper(): Stepper<T> = stepper()
}

/** Returns a list corresponding to stepping through all the values in the specified [list]. */
fun <T> Stepper<T>.stepToAll(list: Iterable<Double>): List<T> = list.map { stepTo(it) }

/**
 * Returns a list corresponding to stepping through all the values in the specified [list].
 * This assumes that all values in [list] can be stepped through.
 */
fun <T> Steppable<T>.stepToAll(list: Iterable<Double>): List<T> = stepper().stepToAll(list)
