package org.futurerobotics.jargon.linalg

/*
 * Kotlin extension functions to provide for-each operations on vectors and matrices.
 */

/**
 * Runs an action on every component of [this] vector.
 */
inline fun Vec.forEach(action: (Double) -> Unit) {
    repeat(size) { i ->
        action(this[i])
    }
}

/**
 * Runs an action on every component of [this] vector, with index.
 */
inline fun Vec.forEachIndexed(action: (Int, Double) -> Unit) {
    repeat(size) { i ->
        action(i, this[i])
    }
}

/**
 * Runs an action on every element in [this] matrix.
 */
inline fun Mat.forEach(action: (Double) -> Unit) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            action(this[r, c])
        }
    }
}

/**
 * Runs an action on every element in [this] matrix, with row and column index.
 */
inline fun Mat.forEachIndexed(action: (r: Int, c: Int, Double) -> Unit) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            action(r, c, this[r, c])
        }
    }
}
