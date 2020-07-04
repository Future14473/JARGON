@file:JvmName("MathUnits")
@file:Suppress("KDocMissingDocumentation", "MemberVisibilityCanBePrivate", "unused")

package org.futurerobotics.jargon.math.convert

import org.futurerobotics.jargon.math.TAU
/*
Expressions of units in terms of SI units.
 */
//Time
const val seconds: Double = 1.0
const val s: Double = seconds
const val minutes: Double = 60.0
const val mins: Double = minutes //not min to confuse with min function
const val milliseconds: Double = 1e-3
const val millis: Double = milliseconds
const val ms: Double = milliseconds
const val microseconds: Double = 1e-6
const val micros: Double = microseconds
const val nanoseconds: Double = 1e-9
const val nanos: Double = nanoseconds
const val ns: Double = nanoseconds
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
//Mass
const val kilograms: Double = 1.0
const val kg: Double = kilograms
const val grams: Double = 1 / 1000.0
const val g: Double = grams
/** Pounds, in mass. Not to be confused with pounds of force [poundsForce] */
const val poundsMass: Double = 0.45359237
/** Pounds, in mass. Not to be confused with pounds of force [lbf] */
const val lbm: Double = poundsMass
/** Ounces, in mass. Not to be confused with ounces of force [ouncesForce] */
const val ouncesMass: Double = poundsMass / 16
/** Ounces, in mass. Not to be confused with ounces of force [ozf] */
const val ozm: Double = ouncesMass
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
/** Pounds, in force. Not to be confused with pounds of mass [poundsMass] */
const val poundsForce: Double = 4.44822
/** Pounds, in force. Not to be confused with pounds of mass [lbm] */
const val lbf: Double = poundsForce
/** Ounces, in force. Not to be confused with pounds of mass [ouncesMass] */
const val ouncesForce: Double = poundsForce / 16
/** Ounces, in force. Not to be confused with pounds of mass [ozm] */
const val ozf: Double = ouncesForce
//basic electrical, for fluency
const val volts: Double = 1.0
const val V: Double = volts
const val amperes: Double = 1.0
const val A: Double = amperes
const val ohms: Double = 1.0
//no omega symbol
