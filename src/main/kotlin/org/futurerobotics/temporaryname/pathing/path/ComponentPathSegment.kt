package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Derivatives


/** A path segment that combines a [Curve] with a [HeadingProvider], to create a Path. */
class ComponentPathSegment(private val curve: Curve, private val heading: HeadingProvider) : PathSegment,
    Curve by curve {

    override fun heading(s: Double): Double {
        return heading.getHeading(curve, s)
    }

    override fun headingDeriv(s: Double): Double {
        return heading.getHeadingDeriv(curve, s)
    }

    override fun headingSecondDeriv(s: Double): Double {
        return heading.getHeadingSecondDeriv(curve, s)
    }

    override fun getPointInfo(s: Double): PathPointInfo {
        val curvePointInfo = curve.getPointInfo(s)
        return ComponentPathPointInfo(curvePointInfo, heading.getHeadingInfo(curve, s, curvePointInfo))
    }

    override fun getAllPointInfo(allS: List<Double>): List<PathPointInfo> {
        return curve.getAllPointInfo(allS).zip(allS) { point, s ->
            ComponentPathPointInfo(
                point, heading.getHeadingInfo(curve, s, point)
            )
        }
    }

    private class ComponentPathPointInfo(
        private val curvePointInfo: CurvePointInfo, private val headingVal: Derivatives
    ) : PathPointInfo, CurvePointInfo by curvePointInfo {
        override val heading: Double get() = headingVal.value
        override val headingDeriv: Double get() = headingVal.valueDeriv
        override val headingSecondDeriv: Double get() = headingVal.valueSecondDeriv
    }
}

/** Returns a [ComponentPathSegment] using this curve and the given [heading] [HeadingProvider] */
fun Curve.addHeading(heading: HeadingProvider): ComponentPathSegment = ComponentPathSegment(this, heading)