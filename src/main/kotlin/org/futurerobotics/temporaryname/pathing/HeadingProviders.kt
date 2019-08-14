package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Derivatives
import org.futurerobotics.temporaryname.math.ValueDerivatives
import org.futurerobotics.temporaryname.math.function.MathFunction

/** A [HeadingProvider] that maintains a constant heading [angle] */
class ConstantHeading(private val angle: Double) : HeadingProvider {
    private val derivatives = ValueDerivatives(angle, 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): Derivatives = derivatives
}

/** A [HeadingProvider] that interpolates linearly among two angles along the length of each curve */
class LinearInterpolatedHeading(private val fromAngle: Double, endAngle: Double) : HeadingProvider {
    private val diff = endAngle - fromAngle
    override fun getHeading(point: CurvePoint, s: Double): Derivatives {
        val dcl = diff / point.length
        return ValueDerivatives(fromAngle + s * dcl, dcl, 0.0)
    }
}

/** The singleton [HeadingProvider] that has the heading equal to the curve's tangent angle. Use for non-holonomic drives. */
object TangentHeading : HeadingProvider {
    override fun getHeading(point: CurvePoint, s: Double): Derivatives = object : Derivatives {
        override val value: Double get() = point.tanAngle
        override val valueDeriv: Double get() = point.tanAngleDeriv
        override val valueSecondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle plus [angleOffset]. */
class OffsetTangentHeading(private val angleOffset: Double) : HeadingProvider {
    override fun getHeading(point: CurvePoint, s: Double): Derivatives = object : Derivatives {
        override val value: Double get() = point.tanAngle + angleOffset
        override val valueDeriv: Double get() = point.tanAngleDeriv
        override val valueSecondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: MathFunction) : HeadingProvider {
    override fun getHeading(point: CurvePoint, s: Double): Derivatives = object : Derivatives {
        private val t = s / point.length
        override val value: Double
            get() = function(s / point.length)
        override val valueDeriv: Double
            get() = function.deriv(t)
        override val valueSecondDeriv: Double
            get() = function.secondDeriv(t)
    }
}
