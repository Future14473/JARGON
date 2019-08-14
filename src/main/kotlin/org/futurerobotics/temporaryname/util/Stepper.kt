package org.futurerobotics.temporaryname.util

/**
 * Represents something that can be stepped with a [Stepper].
 */
interface Steppable<in T, out R> {

    /**
     * Gets a stepper stepping through this object.
     */
    fun stepper(): Stepper<T, R>
}

/**
 * Returns a list corresponding to stepping through all the values in the specified [list].
 * This assumes that all values in [list] can be stepped through.
 */
fun <T, R> Steppable<T, R>.stepToAll(list: Iterable<T>): List<R> {
    val stepper = stepper()
    return list.map(stepper::stepTo)
}

/**
 * A [Stepper] is similar to an [Iterator], except that on each iteration an input value of type [T] must be specified
 * as to where to "step"/iterate to.
 *
 * _This is generally used to being able to apply a heuristic that values between
 * steps are close together in some way_
 *
 * This can be used, for example, to avoid doing binary search on every iteration, while also not wasting memory on
 * intermediary lists.
 *
 * hasNextAt(place: T) will return
 */
interface Stepper<in T, out R> {

    /**
     * Gets the next value at [step].
     */
    fun stepTo(step: T): R
}

/**
 * Returns a list corresponding to stepping through all the values in the specified [list].
 * This assumes that all values in [list] can be stepped through, else [NoSuchElementException] maybe thrown
 */
fun <T, R> Stepper<T, R>.stepToAll(list: List<T>): List<R> = list.map(::get)

/**
 * Gets the next value of this [Stepper] at [step]
 */
operator fun <T, R> Stepper<T, R>.get(step: T): R = stepTo(step)

/**
 * Returns a stepper that uses the supplied [mapping] function for stepping.
 */
@Suppress("FunctionName")
inline fun <T, R> Stepper(crossinline mapping: (T) -> R): Stepper<T, R> = object : Stepper<T, R> {
    override fun stepTo(step: T): R = mapping(step)
}

/**
 * Returns a stepper that always can be stepped to for any place, and uses the supplied [stepNext] function
 * that takes in the previous input and current input to step to next.
 *
 * [initialValue] is used as the previousPlace parameter for the first time stepped.
 */
inline fun <T : V, V, R> pastCurrentStepper(
    initialValue: V, crossinline stepNext: (previousPlace: V, curPlace: T) -> R
): Stepper<T, R> = object : Stepper<T, R> {
    private var previousPlace: V = initialValue
    override fun stepTo(step: T): R = stepNext(previousPlace, step).also { previousPlace = step }
}
