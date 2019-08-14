@file:Suppress("KDocMissingDocumentation", "MemberVisibilityCanBePrivate", "unused")

package org.futurerobotics.temporaryname.math

/**
 * Used for fluent conversion/expression of units. PLS USE ME.
 *
 * Standard units are SI units.
 */

object MathUnits {
    //Time
    const val seconds: Double = 1.0
    const val minutes: Double = 60.0
    const val milliseconds: Double = 1e-3
    const val microseconds: Double = 1e-6
    const val nanoseconds: Double = 1e-9

    //Distance
    const val meters: Double = 1.0
    const val centimeters: Double = 1 / 100.0
    const val inches: Double = centimeters * 2.54
    const val feet: Double = inches * 12
    const val yards: Double = feet * 3

    //Weight
    const val kilograms: Double = 1.0
    const val grams: Double = 1 / 1000.0
    const val pounds: Double = 2.20462
    const val ounces: Double = pounds / 16

    //angles
    const val radians: Double = 1.0
    const val revolutions: Double = TAU
    const val degrees: Double = revolutions / 360
}
