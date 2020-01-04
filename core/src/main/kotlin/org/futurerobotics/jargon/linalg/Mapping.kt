@file:JvmName("MatMap")

package org.futurerobotics.jargon.linalg

/**
 * Creates a new matrix with all the values mapped through the given [map] function.
 */
inline fun Mat.map(func: (Double) -> Double): Mat = zeroMat(rows, cols).apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = func(this@map[r, c])
        }
    }
}
//vec map exists
/**
 * Creates a new matrix with all the values mapped through the given [map] function, indexed
 */
inline fun Mat.mapIndexed(func: (r: Int, c: Int, Double) -> Double): Mat = zeroMat(rows, cols).apply {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = func(r, c, this@mapIndexed[r, c])
        }
    }
}

/**
 * Creates a new vector with all the values mapped through the given [map] function, indexed
 */
inline fun Vec.mapIndexed(func: (index: Int, Double) -> Double): Vec = zeroVec(size).apply {
    repeat(size) {
        this[it] = func(it, this@mapIndexed[it])
    }
}

/**
 * Maps the values of this matrix to itself.
 */
inline fun Mat.mapToSelf(func: (Double) -> Double) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = func(this[r, c])
        }
    }
}
//vec map to self exists
/**
 * Combines this matrix with the [other] matrix, through the given func, overriding the values in `this` matrix.
 */
inline fun Mat.combineToSelf(other: Mat, func: Double.(Double) -> Double) {
    require(rows == other.rows && cols == other.cols) { "Dimension mismatch" }
    repeat(rows) { r ->
        repeat(cols) { c ->
            this[r, c] = this[r, c].func(other[r, c])
        }
    }
}

/**
 * Combines this matrix with the [other] vector, through the given func, overriding the values in `this` matrix.
 */
inline fun Vec.combineToSelf(other: Vec, func: Double.(Double) -> Double) {
    require(size == other.size) { "Dimension mismatch" }
    repeat(size) {
        this[it] = this[it].func(other[it])
    }
}

/**
 * Maps a list into a vector using the given [transform] function.
 */
inline fun <T> List<T>.mapToVec(transform: (T) -> Double): Vec =
    zeroVec(size).also { v ->
        forEachIndexed { index, t ->
            v[index] = transform(t)
        }
    }
