package org.futurerobotics.jargon.linalg

import kotlin.math.sign

fun sign(vec: Vec): Vec = vec.map { sign(it) }


fun Vec.toList(): List<Double> = List(dimension) { this[it] }