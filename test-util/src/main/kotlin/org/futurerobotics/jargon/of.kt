package org.futurerobotics.jargon

/**
 * To make formatting less painful (kotlin only), especially with [koma.mat]
 *
 * Usage: mat[of the
 *      (...)
 *      ]
 */
object of {
    inline infix fun <T> the(value: T) = value
}