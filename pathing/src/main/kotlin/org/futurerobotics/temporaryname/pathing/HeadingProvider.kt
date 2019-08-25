package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Derivatives

/**
 * Provides heading to complete a [Curve] into a [Path]
 * @see ComponentPath
 */
interface HeadingProvider {

    /**
     * Gets a heading's derivatives at the point [s] units along the curve, using info provided by the [CurvePoint] [point]
     */
    fun getHeading(point: CurvePoint, s: Double): Derivatives
}
