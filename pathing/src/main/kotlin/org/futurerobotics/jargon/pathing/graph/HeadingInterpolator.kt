package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.pathing.*

/**
 * Provider for a [HeadingProvider], that interpolates between two headings, given a curve.
 *
 * This is assumed to be immutable.
 *
 * This is separate from [HeadingProvider] so we can have those sweet immutable data structures; and make
 * single responsibility (or something close to it) easier.
 */
interface HeadingInterpolator {

    /**
     * Gets a [HeadingProvider] that will interpolate heading for the given [curve] between the [startHeading] and
     * [endHeading].
     *
     * This is also allowed to throw exceptions if such is not possible.
     */
    fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider
}

/** Convenience extension function for creating a [CurveHeadingPath] with this curve and a [HeadingInterpolator] */
fun Curve.addHeading(interpolator: HeadingInterpolator, startHeading: Double, endHeading: Double): Path =
    addHeading(interpolator.getHeadingProvider(this, startHeading, endHeading))
