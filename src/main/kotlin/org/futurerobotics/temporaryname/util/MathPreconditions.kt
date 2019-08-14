package org.futurerobotics.temporaryname.util

/**
 * [require]s the [value] to be Finite, else throws an [IllegalArgumentException] with the [lazyMessage]
 */
inline fun requireFinite(
    value: Double, lazyMessage: (Double) -> String = { "A double value ($value) was not finite" }
) {
    if (!value.isFinite()) throw IllegalArgumentException(lazyMessage(value))
}

/**
 * Shorthand for [requireFinite] with a simple message using [name]
 */
@Suppress("NOTHING_TO_INLINE")
inline fun requireFiniteNamed(value: Double, name: String) {
    requireFinite(value) { "$name ($it) should be finite" }
}
