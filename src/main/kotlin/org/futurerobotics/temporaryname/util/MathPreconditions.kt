package org.futurerobotics.temporaryname.util


/**
 * [require]s the [value] to not be NaN, else throws an [IllegalArgumentException] with the [lazyMessage]
 */
inline fun requireNotNaN(value: Double, lazyMessage: () -> String = { "A double value was NaN" }) {
    if (value.isNaN()) throw IllegalArgumentException(lazyMessage())
}

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
inline fun requireFiniteNamed(value: Double, name: String) {
    requireFinite(value) { "$name ($it) should be finite" }
}
