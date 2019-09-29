/**
 * Math utilities and calculations.
 */
@file:JvmName("MathUtil")

package org.futurerobotics.jargon.math

import kotlin.math.*

/**
 * TAU, which is 2*[PI]
 */
const val TAU: Double = 2 * PI
/**
 * The leniency given to Double comparison in [epsEq] and related functions.
 */
const val EPSILON: Double = 1e-8

/**
 * Short for "epsilon equals"
 *
 * If [this] is equal to [other], with a difference tolerance of [EPSILON] to account for floating-point errors.
 */
infix fun Double.epsEq(other: Double): Boolean = abs(this - other) < EPSILON

/**
 * Returns [this] if not [isNaN], else the value given by [alternate]
 */
inline fun Double.notNaNOrElse(alternate: () -> Double): Double = if (isNaN()) alternate() else this

/** Returns the average of [a] and [b] */
fun avg(a: Double, b: Double): Double = (a + b) / 2

/** Returns the average of [a], [b], and [c] */
fun avg(a: Double, b: Double, c: Double): Double = (a + b + c) / 3

/** Returns the greatest of [a], [b], and [c]. If any value is `NaN`, the result is `NaN` */
fun max(a: Double, b: Double, c: Double): Double = max(max(a, b), c)

/** Returns the greatest of [a], [b], [c], and [d]. If any value is `NaN`, the result is `NaN` */
fun max(a: Double, b: Double, c: Double, d: Double): Double = max(max(a, b), max(c, d))

/** Returns the maximum difference between [a], [b] and [c]. If any value is `NaN`, the result is 'NaN` */
fun maxDiff(a: Double, b: Double, c: Double): Double = max(abs((a - b)), abs((b - c)), abs((c - a)))

/** [this] value squared.*/
fun Double.squared(): Double = this * this

/** [this] value squared.*/
fun Int.squared(): Int = this * this

/** [this] value cubed.*/
fun Double.cubed(): Double = this.pow(3)

/** The distance from this value to another value. Equal to `abs(this-v)` */
infix fun Double.distTo(v: Double): Double = abs(this - v)

/** The distance from this value to another value. Equal to `abs(this-v)` */
infix fun Int.distTo(v: Int): Int = abs(this - v)

/** Normalizes an angle value to be between `-PI` and `PI` */
fun angleNorm(angle: Double): Double = angle - TAU * floor((angle + PI) / TAU)

/** Calculates the curvature at a point on a 2d curve given its first and second derivatives */
fun calcCurvature(v: Vector2d, a: Vector2d): Double = (v cross a) / v.lengthPow(3.0)

/** Calculates the curvature's derivative at a point on a 2d curve given its first, second, and third derivatives */
fun calcCurvatureDeriv(v: Vector2d, a: Vector2d, j: Vector2d): Double =
    (v cross j) / v.lengthPow(3.0) - 3 * (v cross a) * (v dot a) / v.lengthPow(5.0)

/**
 * Performs the common operation `sin(x)/x`.
 */
fun sinc(x: Double): Double = when {
    x epsEq 0.0 -> 1 - x.squared() / 6
    else -> sin(x) / x
}

/**
 * Performs the common operation `(1-cos x)/x`
 */
fun cosc(x: Double): Double = when {
    x epsEq 0.0 -> x / 2
    else -> (1 - cos(x)) / x
}