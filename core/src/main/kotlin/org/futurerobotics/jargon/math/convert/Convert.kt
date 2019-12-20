/**
 * Used for easy and fluent conversion/expression of units.
 *
 * Standard units are SI units, and other values represent the same value in SI units.
 *
 * For example, `3 * inches` is 3 inches in SI units, or 0.0762 m.
 *
 * Beware of forces, as 1 pound != 1 pound-force; use `lbf` instead of `lb` where appropriate.
 *
 * However, beware when dimensional analysis; the same does not follow.
 */
@file:JvmName("MathUnits")
@file:Suppress("KDocMissingDocumentation", "MemberVisibilityCanBePrivate", "unused")

package org.futurerobotics.jargon.math.convert

import org.futurerobotics.jargon.math.TAU

//Time
const val seconds: Double = 1.0
const val s: Double = seconds
const val minutes: Double = 60.0
const val mins: Double = minutes
const val milliseconds: Double = 1e-3
const val millis: Double = milliseconds
const val microseconds: Double = 1e-6
const val micros: Double = microseconds
const val nanoseconds: Double = 1e-9
const val nanos: Double = nanoseconds
//Distance
const val meters: Double = 1.0
const val m: Double = meters
const val centimeters: Double = 1 / 100.0
const val cm: Double = centimeters
const val millimeters: Double = meters / 1000.0
const val mm: Double = meters / 1000.0
const val inches: Double = centimeters * 2.54
const val `in`: Double = inches
const val feet: Double = inches * 12
const val ft: Double = feet
const val yards: Double = feet * 3
const val yd: Double = yards
//Weight
const val kilograms: Double = 1.0
const val kg: Double = kilograms
const val grams: Double = 1 / 1000.0
const val g: Double = grams
const val pounds: Double = 0.45359237
const val lbs: Double = pounds
const val ounces: Double = pounds / 16
const val oz: Double = ounces
//Angles
const val radians: Double = 1.0
const val rad: Double = radians
const val revolutions: Double = TAU
const val rev: Double = revolutions
const val degrees: Double = revolutions / 360
const val deg: Double = degrees
//Force
const val newtons: Double = kg * m / s
const val N: Double = newtons
const val poundsForce: Double = 4.44822
const val lbf: Double = poundsForce
const val ouncesForce: Double = poundsForce / 16
const val ozf: Double = ouncesForce
//basic electrical, for fluency
const val volts: Double = 1.0
const val V: Double = volts
const val amperes: Double = 1.0
const val A: Double = amperes
const val ohms: Double = 1.0
