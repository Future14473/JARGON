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
 * Returns a [Steppable] that uses the given [stepper] function to provide a stepper for stepping.
 */
inline fun <T, R> Steppable(crossinline stepper: () -> Stepper<T, R>) = object :
    Steppable<T, R> {
    override fun stepper(): Stepper<T, R> = stepper()
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
 * Returns a stepper that uses the supplied [step] function for stepping.
 */
@Suppress("FunctionName")
inline fun <T, R> Stepper(crossinline step: (T) -> R): Stepper<T, R> = object :
    Stepper<T, R> {
    override fun stepTo(step: T): R = step(step)
}

/**
 * Returns a new Steppable which maps the output of [this] stepper through the given [outputMapping].
 */
inline fun <T, V, R> Steppable<T, V>.mapOutput(crossinline outputMapping: (V) -> R): Steppable<T, R> =
    mapInputOutput({ it }, outputMapping)

/**
 * Returns a new Steppable which first maps the [step] parameter through the given [inputMapping]
 */
inline fun <T, V, R> Steppable<V, R>.mapInput(crossinline inputMapping: (T) -> V): Steppable<T, R> =
    mapInputOutput(inputMapping, { it })

/**
 * Returns a new Steppable which first maps the [step] parameter through the given [inputMapping] AND then the output
 * through the given [outputMapping]
 */
inline fun <T1, T2, R1, R2> Steppable<T1, R1>.mapInputOutput(
    crossinline inputMapping: (T2) -> T1,
    crossinline outputMapping: (R1) -> R2
): Steppable<T2, R2> =
    Steppable {
        val stepper = stepper()
        Stepper<T2, R2> {
            outputMapping(stepper.stepTo(inputMapping(it)))
        }
    }