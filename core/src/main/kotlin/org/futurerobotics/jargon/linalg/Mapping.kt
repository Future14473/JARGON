package org.futurerobotics.jargon.linalg

/**
 * Creates a new matrix with all the values mapped through the given [map] function.
 */
inline fun Mat.map(func: (Double) -> Double): Mat =
    Mat(rows, cols) { r, c ->
        func(this@map[r, c])
    }
//vec map exists
/**
 * Creates a new matrix with all the values mapped through the given [map] function, indexed
 */
inline fun Mat.mapIndexed(func: (r: Int, c: Int, Double) -> Double): Mat =
    Mat(rows, cols) { r, c ->
        func(r, c, this@mapIndexed[r, c])
    }

/**
 * Creates a new vector with all the values mapped through the given [map] function, indexed
 */
inline fun Vec.mapIndexed(func: (index: Int, Double) -> Double): Vec =
    Vec(size) {
        func(it, this@mapIndexed[it])
    }

/**
 * Maps a list into a vector using the given [transform] function.
 */
inline fun <T> List<T>.mapToVec(transform: (T) -> Double): Vec {
    val iterator = iterator()
    return Vec(size) {
        check(iterator.hasNext())
        transform(iterator.next())
    }
}
