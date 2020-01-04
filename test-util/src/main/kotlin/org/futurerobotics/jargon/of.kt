package org.futurerobotics.jargon

/**
 * To make formatting slightly less painful (kotlin only).
 *
 * Usage example: mat[of the
 *      (...) end
 *      (...)
 *      ]
 */
@Suppress("ClassName", "NOTHING_TO_INLINE")
object of {

    /** returns [value]. */
    inline infix fun <T> the(value: T): T = value
}
