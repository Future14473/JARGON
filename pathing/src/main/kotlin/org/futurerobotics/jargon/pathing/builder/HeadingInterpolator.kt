package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.*

/**
 * Provider for a [HeadingProvider], that _may attempt to_ interpolate between two headings, given a curve.
 *
 * This is assumed to be immutable.
 *
 * This is separate from [HeadingProvider] so we can have those sweet immutable data structures.
 */
interface HeadingInterpolator {

    /**
     * Gets a [HeadingProvider] that _may_ interpolate heading for the given [curve] between the [startHeading] and
     * [endHeading].
     */
    fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider

    /**
     * Adds a [HeadingProvider] to the given [curve] that _may_ interpolate between [startHeading] and [endHeading].
     */
    fun addHeadingTo(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ) = curve.addHeading(getHeadingProvider(curve, startHeading, endHeading))
}

/** Convenience extension function for creating a [CurveHeadingPath] with this curve and a [HeadingInterpolator] */
fun Curve.addHeading(interpolator: HeadingInterpolator, startHeading: Double, endHeading: Double): Path =
    addHeading(interpolator.getHeadingProvider(this, startHeading, endHeading))
