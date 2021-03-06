package org.futurerobotics.jargon.math

import kotlin.math.*

/** TAU, which is 2*[PI] */
const val TAU: Double = 2 * PI

/** The default leniency given to Double comparison in [epsEq] and related functions. */
const val EPSILON: Double = 1e-8

/**
 * Short for "epsilon equals": returns if [this] is equal to [other], with a difference tolerance of [EPSILON] to
 * account for floating-point errors.
 */
infix fun Double.epsEq(other: Double): Boolean = abs(this - other) < EPSILON

/**
 * Short for "epsilon equals": returns if [this] is equal to [other], with a difference tolerance of a configurable
 * [epsilon] to account for floating-point errors.
 */
fun Double.epsEq(other: Double, epsilon: Double): Boolean = abs(this - other) < epsilon

/**
 * Returns [this] if not [isNaN], else the value given by [alternate]
 */
inline fun Double.ifNan(alternate: () -> Double): Double = if (isNaN()) alternate() else this

/**
 * Returns [this] if [isFinite], else the value given by [alternate]
 */
inline fun Double.ifNonFinite(alternate: () -> Double): Double = if (!isFinite()) alternate() else this

/** Returns the average of [a] and [b] */
fun avg(a: Double, b: Double): Double = (a + b) / 2

/** Returns the average of [a], [b], and [c] */
fun avg(a: Double, b: Double, c: Double): Double = (a + b + c) / 3

/** The distance from this value to another value. Equal to `abs(this-v)` */
infix fun Double.distTo(v: Double): Double = abs(this - v)

/** Normalizes an angle value to be between `-PI` and `PI` */
fun angleNorm(angle: Double): Double = angle - TAU * round(angle / TAU)

/**
 * The function `sin(x)/x`, working well with numbers near zero.
 */
fun sinc(x: Double): Double = when {
    x.epsEq(0.0, 1e-100) -> 1 - x * x / 6
    else -> sin(x) / x
}

/**
 * The function `sin(x)/x`, working well with numbers near zero.
 */
fun cosc(x: Double): Double = when {
    x.epsEq(0.0, 1e-100) -> x / 2
    else -> (1 - cos(x)) / x
}
