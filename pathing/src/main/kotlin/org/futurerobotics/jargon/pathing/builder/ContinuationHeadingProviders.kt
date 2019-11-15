package org.futurerobotics.jargon.pathing.builder

import org.futurerobotics.jargon.math.LinearMotionState
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.pathing.*

/**
 * A [ContinuationHeadingProvider] that will linearly interpolate heading from the previous heading _by_ the given
 * `turnBy` angle to the end of the curve.
 */
class TurnByContinuation(private val turnBy: Double) : ContinuationHeadingProvider {

    init {
        require(turnBy.isFinite()) { "Angle ($turnBy) must be finite" }
    }

    override fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider {
        val fromHeading = lastPoint.headingMotionState()
        return PolyInterpolatedHeading(
            fromHeading,
            LinearMotionState(fromHeading.value + turnBy, 0.0, 0.0)
        )
    }
}

/**
 * A [ContinuationHeadingProvider] that will linearly interpolate the heading so the final heading at the end of the
 * curve _will be_ the `turnTo` angle.
 */
class TurnToContinuation(turnTo: Double) : ContinuationHeadingProvider {

    private val turnTo = angleNorm(turnTo)

    init {
        require(turnTo.isFinite()) { "Angle ($turnTo) must be finite" }
    }

    override fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider {
        val fromHeading = lastPoint.headingMotionState()
        return PolyInterpolatedHeading(
            fromHeading,
            LinearMotionState(fromHeading.value + angleNorm(turnTo - fromHeading.value), 0.0, 0.0)
        )
    }
}

/**
 * A [ContinuationHeadingProvider] that will linearly interpolate heading from the previous heading _by_ the given
 * [turnBy] angle to the end of the curve.
 */
class TurnOffsetByContinuation(private val turnBy: Double) : ContinuationHeadingProvider {

    init {
        require(turnBy.isFinite()) { "Angle ($turnBy) must be finite" }
    }

    override fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider {

        val fromOffset = lastPoint.headingMotionState() - lastPoint.tanAngleMotionState()
        return PolyInterpolatedTangentHeading(
            fromOffset,
            LinearMotionState(fromOffset.value + turnBy, 0.0, 0.0)
        )
    }
}

/**
 * A [ContinuationHeadingProvider] that will linearly interpolate the heading so the final heading at the end of the
 * curve _will be_ the `turnTo` angle.
 */
class TurnOffsetToContinuation(turnTo: Double) : ContinuationHeadingProvider {

    private val turnTo = angleNorm(turnTo)

    init {
        require(turnTo.isFinite()) { "Angle ($turnTo) must be finite" }
    }

    override fun getHeadingProvider(lastPoint: PathPoint, curve: Curve): HeadingProvider {
        val fromOffset = lastPoint.headingMotionState() - lastPoint.tanAngleMotionState()
        return PolyInterpolatedTangentHeading(
            fromOffset,
            LinearMotionState(fromOffset.value + angleNorm(turnTo - fromOffset.value), 0.0, 0.0)
        )
    }
}

