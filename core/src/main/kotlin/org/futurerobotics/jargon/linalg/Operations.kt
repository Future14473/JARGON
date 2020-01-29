package org.futurerobotics.jargon.linalg

/**
 * Runs an action on every element in [this] vector.
 */
inline fun Vec.forEach(action: (Double) -> Unit) {
    repeat(size) { i ->
        action(this[i])
    }
}

/**
 * Runs an action on every element in [this] vector, indexed.
 */
inline fun Vec.forEachIndexed(action: (Int, Double) -> Unit) {
    repeat(size) { i ->
        action(i, this[i])
    }
}

/**
 * Runs an action on every element in [this] matrix, row-wise first..
 */
inline fun Mat.forEach(action: (Double) -> Unit) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            action(this[r, c])
        }
    }
}

/**
 * Runs an action on every element in [this] vector, indexed.
 */
inline fun Mat.forEachIndexed(action: (r: Int, c: Int, Double) -> Unit) {
    repeat(rows) { r ->
        repeat(cols) { c ->
            action(r, c, this[r, c])
        }
    }
}
