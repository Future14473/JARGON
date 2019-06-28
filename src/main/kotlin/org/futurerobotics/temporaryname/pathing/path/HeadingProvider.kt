package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Derivatives
import org.futurerobotics.temporaryname.math.ValueDerivatives

/**
 * Provides heading to complete a [Curve] into a [PathSegment]
 * @see ComponentPathSegment
 */
interface HeadingProvider {
    /**
     * Gets the heading at the point [s] units along the [curve]
     */
    fun getHeading(curve: Curve, s: Double): Double

    /**
     * Gets the heading's derivative at the point [s] units along the [curve]
     */
    fun getHeadingDeriv(curve: Curve, s: Double): Double

    /**
     * Gets the heading's second derivative at the point [s] units along the [curve]
     */
    fun getHeadingSecondDeriv(curve: Curve, s: Double): Double

    /**
     * Gets a heading's derivatives (heading, headingDeriv, and headingSecondDeriv combined)
     * at the point [s] units along the [curve]. Use the [CurvePointInfo] [point] where possible.
     */
    fun getHeadingInfo(curve: Curve, s: Double, point: CurvePointInfo): Derivatives =
        ValueDerivatives(getHeading(curve, s), getHeadingDeriv(curve, s), getHeadingSecondDeriv(curve, s))
}
