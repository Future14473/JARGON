package org.futurerobotics.jargon.linalg
/*
 * Kotlin extension functions to provide mappings functions of values in vectors and matrices.
 */

/**
 * Creates a new matrix with all the values mapped through the given [transform] function.
 */
inline fun Mat.map(transform: (Double) -> Double): Mat =
    Mat(rows, cols) { r, c ->
        transform(this@map[r, c])
    }

//RealVector.map already exists.

/**
 * Creates a new matrix with all the values mapped through the given [transform] function, indexed
 */
inline fun Mat.mapIndexed(transform: (r: Int, c: Int, Double) -> Double): Mat =
    Mat(rows, cols) { r, c ->
        transform(r, c, this@mapIndexed[r, c])
    }

/**
 * Creates a new vector with all the values mapped through the given [transform] function, indexed
 */
inline fun Vec.mapIndexed(transform: (index: Int, Double) -> Double): Vec =
    Vec(size) {
        transform(it, this@mapIndexed[it])
    }

/**
 * Maps a list into a vector using the given [transform] function.
 */
inline fun <T> List<T>.mapToVec(transform: (T) -> Double): Vec {
    val iterator = iterator()
    return Vec(size) {
        transform(iterator.next())
    }
}
