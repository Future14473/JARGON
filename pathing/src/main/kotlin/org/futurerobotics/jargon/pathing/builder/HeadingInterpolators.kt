package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.pathing.*

/**
 * A [HeadingInterpolator] that simply yields a given [interpolator].
 */
class ProviderInterpolator(private val interpolator: HeadingProvider) : HeadingInterpolator {

    override fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider = interpolator
}

/**
 * A [HeadingInterpolator] only outputs tangent heading, with an optional constant offset.
 *
 * This is the recommended heading for non-holonomic drives.
 */
class TangentInterpolator
@JvmOverloads constructor(offset: Double = 0.0) :
    HeadingInterpolator {

    private val heading = TangentHeading(offset)

    override fun getHeadingProvider(
        curve: Curve,
        startHeading: Double,
        endHeading: Double
    ): HeadingProvider = heading
}

/**
 * A [HeadingInterpolator] that linearly interpolates the _offset to the tangent angle_.
 */
class LinearOffsetInterpolator : HeadingInterpolator {

    override fun getHeadingProvider(curve: Curve, startHeading: Double, endHeading: Double): HeadingProvider =
        LinInterpTangentHeading(
            startHeading - curve.startPoint().tanAngle,
            endHeading - curve.endPoint().tanAngle
        )
}

/**
 * A [HeadingInterpolator] that linearly interpolates heading.
 */
open class LinearInterpolator : HeadingInterpolator {

    final override fun getHeadingProvider(curve: Curve, startHeading: Double, endHeading: Double): HeadingProvider =
        LinInterpHeading(startHeading, endHeading)

    companion object : LinearInterpolator()
}
