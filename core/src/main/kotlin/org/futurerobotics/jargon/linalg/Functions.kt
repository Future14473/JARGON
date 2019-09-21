package org.futurerobotics.jargon.linalg

import kotlin.math.sign

fun sign(vec: Vec): Vec = vec.map { sign(it) }

infix fun Vec.emul(other: Vec): Vec = this.ebeMultiply(other)

fun Vec.toList(): List<Double> = List(dimension) { this[it] }