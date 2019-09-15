package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Derivatives
import org.futurerobotics.temporaryname.util.Stepper

/**
 * Provides heading info to complete a [Curve] into a [Path]
 * @see ComponentPath
 */
interface HeadingProvider {

    /**
     * Gets a heading's derivatives at the point [s] units along the curve, using info provided by the [CurvePoint] [point]
     */
    fun getHeading(point: CurvePoint, s: Double): Derivatives<Double>
}

/**
 * A path that combines a [Curve] with a [HeadingProvider], to create a Path.
 */
class ComponentPath(internal val curve: Curve, private val heading: HeadingProvider) : Path {

    override val length: Double get() = curve.length
    override fun pointAt(s: Double): PathPoint {
        val point = curve.pointAt(s)
        return ComponentPathPoint(
            point,
            heading.getHeading(point, s)
        )
    }

    override fun stepper(): Stepper<Double, PathPoint> {
        val curveStepper = curve.stepper()
        return Stepper { s ->
            val point = curveStepper.stepTo(s)
            ComponentPathPoint(
                point,
                heading.getHeading(point, s)
            )
        }
    }

    private class ComponentPathPoint(
        private val curvePoint: CurvePoint, private val headingVal: Derivatives<Double>
    ) : PathPoint, CurvePoint by curvePoint {

        override val heading: Double get() = headingVal.value
        override val headingDeriv: Double get() = headingVal.deriv
        override val headingSecondDeriv: Double get() = headingVal.secondDeriv
    }
}

/** Convenience extension function for creating a [ComponentPath] with this curve and a [HeadingProvider] */
fun Curve.addHeading(heading: HeadingProvider): ComponentPath =
    ComponentPath(this, heading)
