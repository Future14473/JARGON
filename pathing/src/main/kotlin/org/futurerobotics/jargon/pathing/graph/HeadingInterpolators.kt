package org.futurerobotics.jargon.pathing.graph

import org.futurerobotics.jargon.pathing.*

/**
 * A [HeadingInterpolator] only outputs tangent heading.
 *
 * This is the recommended heading for non-holonomic drives.
 */
open class TangentInterpolator : HeadingInterpolator {

    final override fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider = TangentHeading

    companion object : TangentInterpolator()
}

/**
 * A [HeadingInterpolator] that provides quintic polynomial interpolated for the _offset_ to the tangent angle.
 */
open class LinearOffsetInterpolator : HeadingInterpolator {

    final override fun getHeadingProvider(curve: Curve, startHeading: Double, endHeading: Double): HeadingProvider =
        LinearlyInterpolatedTangentHeading(
            startHeading - curve.startPoint().tanAngle,
            endHeading - curve.endPoint().tanAngle
        )

    companion object : LinearOffsetInterpolator()
}

/**
 * A [HeadingInterpolator] that provides linearly interpolated headings.
 */
open class LinearInterpolator : HeadingInterpolator {

    final override fun getHeadingProvider(curve: Curve, startHeading: Double, endHeading: Double): HeadingProvider =
        LinearlyInterpolatedHeading(startHeading, endHeading)

    companion object : LinearInterpolator()
}
