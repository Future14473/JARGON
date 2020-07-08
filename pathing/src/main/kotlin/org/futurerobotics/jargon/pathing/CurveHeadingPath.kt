@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.RealMotionState
import org.futurerobotics.jargon.util.Stepper

/**
 * Provides heading to a [Curve] to form a [Path].
 * @see CurveHeadingPath
 * @see addHeading
 */
interface HeadingProvider {

    /**
     * Gets a heading's value and derivatives at the point [s] units along the curve, using info provided by
     * the [CurvePoint] [point]
     */
    fun getHeading(point: CurvePoint, s: Double): RealMotionState
}

/**
 * A path that combines a [Curve] with a [HeadingProvider], to create a [Path].
 *
 * @param curve the curve used
 * @param heading the heading used
 */
class CurveHeadingPath(val curve: Curve, val heading: HeadingProvider) : Path {

    override val length: Double get() = curve.length
    override val stopPoints: Set<Double> get() = curve.stopPoints

    override fun pointAt(s: Double): PathPoint {
        val point = curve.pointAt(s)
        return Point(point, heading.getHeading(point, s))
    }

    override fun stepper(): Stepper<PathPoint> {
        val curveStepper = curve.stepper()
        return Stepper { s ->
            val point = curveStepper.stepTo(s)
            Point(point, heading.getHeading(point, s))
        }
    }

    private class Point(
        curvePoint: CurvePoint,
        heading: RealMotionState
    ) : PathPoint, CurvePoint by curvePoint {

        override val heading: Double = heading.value
        override val headingDeriv: Double = heading.deriv
        override val headingSecondDeriv: Double = heading.secondDeriv
    }
}

/** Convenience extension function for creating a [CurveHeadingPath] with this curve and a [HeadingProvider] */
fun Curve.addHeading(heading: HeadingProvider): CurveHeadingPath = CurveHeadingPath(this, heading)
