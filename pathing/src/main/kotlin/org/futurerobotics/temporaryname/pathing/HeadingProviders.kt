package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Derivatives
import org.futurerobotics.temporaryname.math.ValueDerivatives
import org.futurerobotics.temporaryname.math.function.MathFunction

/** A [HeadingProvider] that maintains a constant heading [angle] */
class ConstantHeading(private val angle: Double) : HeadingProvider {

    private val derivatives = ValueDerivatives(angle, 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): Derivatives<Double> = derivatives
}

/** A [HeadingProvider] that interpolates linearly among two angles along the length of each curve */
class LinearInterpolatedHeading(private val fromAngle: Double, endAngle: Double) :
    HeadingProvider {

    private val diff = endAngle - fromAngle
    override fun getHeading(point: CurvePoint, s: Double): Derivatives<Double> {
        val dcl = diff / point.length
        return ValueDerivatives(fromAngle + s * dcl, dcl, 0.0)
    }
}

/** The singleton [HeadingProvider] that has the heading equal to the curve's tangent angle. Use for non-holonomic drives. */
object TangentHeading : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): Derivatives<Double> = object : Derivatives<Double> {
        override val value: Double get() = point.tanAngle
        override val deriv: Double get() = point.tanAngleDeriv
        override val secondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle plus [angleOffset]. */
class OffsetTangentHeading(private val angleOffset: Double) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): Derivatives<Double> = object : Derivatives<Double> {
        override val value: Double get() = point.tanAngle + angleOffset
        override val deriv: Double get() = point.tanAngleDeriv
        override val secondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: MathFunction) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): Derivatives<Double> = object : Derivatives<Double> {
        private val t = s / point.length
        override val value: Double
            get() = function(s / point.length)
        override val deriv: Double
            get() = function.deriv(t)
        override val secondDeriv: Double
            get() = function.secondDeriv(t)
    }
}
