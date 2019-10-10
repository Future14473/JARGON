package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.MotionState2
import org.futurerobotics.jargon.mechanics.MotionState3
import org.futurerobotics.jargon.mechanics.ValueMotionState3

/**
 * Attempts to convert the input into a [MotionState3] representation of <T>.
 *
 * @param zero the value of zero to fill in missing values
 * @throws ClassCastException if impossible.
 */
@Suppress("UNCHECKED_CAST")
@JvmName("toMotionState3Inline")
inline fun <reified T : Any> Any.castToMotionState3(zero: T): MotionState3<T> = when (this) {
    !is MotionState2<*> -> ValueMotionState3(this as T, zero, zero)
    is MotionState3<*> -> this as MotionState3<T>
    else -> ValueMotionState3(s as T, v as T, zero)
}

/**
 * Attempts to convert this value into a [MotionState3] representation of <T>.
 *
 *
 * @param zero the value of zero to fill in missing values
 * Not that this is slightly less safe as the generic class of [T] is not checked.
 */
@Suppress("UNCHECKED_CAST")
@JvmName("castToMotionState3")
fun <T : Any> Any.castToMotionState3NoInline(zero: T): MotionState3<T> = when (this) {
    !is MotionState2<*> -> ValueMotionState3(this as T, zero, zero)
    is MotionState3<*> -> this as MotionState3<T>
    else -> ValueMotionState3(s as T, v as T, zero)
}

/**
 * Attempts to convert this value to a List<Double>.
 *
 * Allowed types: List<Double>, DoubleArray/double[], [Vec].
 *
 * @throws ClassCastException if impossible.
 */
@Suppress("UNCHECKED_CAST")
fun Any.castToDoubleList(): List<Double> = when (this) {
    is List<*> -> this as List<Double>
    is DoubleArray -> this.asList()
    is Vec -> this.asList()
    else -> throw ClassCastException("Given item $this can not be interpreted as a Double list")
}

/**
 * Attempts to convert this value to a [Vec]
 *
 * Allowed types: List<Double>, DoubleArray/double[], [Vec].
 *
 * Changing the returned vec may or may not change the underlying value, so use for read only.
 *
 * @throws ClassCastException if impossible.
 */
@Suppress("UNCHECKED_CAST")
fun Any.castToVec(): Vec = when (this) {
    is List<*> -> createVec(this as List<Double>)
    is DoubleArray -> createVec(this, false)
    is Vec -> this
    else -> throw ClassCastException("Given item $this can not be interpreted as a Vector")
}