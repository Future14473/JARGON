package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.*

/**
 * Given a curve and target start/end headings, creates [HeadingProvider]s for that curve.
 *
 * It may also choose to ignore the target headings directly.
 *
 * This is separate from [HeadingProvider] so we can have those sweet immutable data structures.
 */
interface HeadingInterpolator {

    /**
     * Gets a [HeadingProvider] for the given [curve], that may interpolate between the [startHeading] and
     * [endHeading].
     */
    fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider

    /**
     * Gets a [HeadingProvider] for the given [curve], that may interpolate between the [startHeading] and
     * [endHeading].
     */
    fun addHeadingTo(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): CurveHeadingPath = curve.addHeading(getHeadingProvider(curve, startHeading, endHeading))
}

/** Convenience extension function for creating a [CurveHeadingPath] with this curve and a [HeadingInterpolator] */
fun Curve.addHeading(interpolator: HeadingInterpolator, startHeading: Double, endHeading: Double): Path =
    addHeading(interpolator.getHeadingProvider(this, startHeading, endHeading))
