package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Derivatives
import org.futurerobotics.temporaryname.math.ValueDerivatives
import org.futurerobotics.temporaryname.math.function.MathFunction

/** A [HeadingProvider] that maintains a constant heading [angle] */
class ConstantHeading(private val angle: Double) : HeadingProvider {
    private val derivatives = ValueDerivatives(angle, 0.0, 0.0)

    override fun getHeading(curve: Curve, s: Double): Double = angle
    override fun getHeadingDeriv(curve: Curve, s: Double): Double = 0.0

    override fun getHeadingSecondDeriv(curve: Curve, s: Double): Double = 0.0

    override fun getHeadingInfo(
        curve: Curve, s: Double, point: CurvePointInfo
    ): Derivatives = derivatives
}

/** A [HeadingProvider] that interpolates linearly among two angles along the length of each curve */
class LinearInterpolatedHeading(private val fromAngle: Double, private val diff: Double) : HeadingProvider {
    override fun getHeading(curve: Curve, s: Double): Double = fromAngle + s * diff / curve.length

    override fun getHeadingDeriv(curve: Curve, s: Double): Double = diff / curve.length

    override fun getHeadingSecondDeriv(curve: Curve, s: Double): Double = 0.0

    override fun getHeadingInfo(
        curve: Curve, s: Double, point: CurvePointInfo
    ): Derivatives {
        val dcl = diff / curve.length
        return ValueDerivatives(fromAngle + s * dcl, dcl, 0.0)
    }
}

/** The singleton [HeadingProvider] that has the heading equal to the curve's tangent angle. Use for non-holonomic drives. */
object TangentHeading : HeadingProvider {
    override fun getHeading(curve: Curve, s: Double): Double = curve.tanAngle(s)

    override fun getHeadingDeriv(curve: Curve, s: Double): Double = curve.tanAngleDeriv(s)

    override fun getHeadingSecondDeriv(curve: Curve, s: Double): Double = curve.tanAngleSecondDeriv(s)

    override fun getHeadingInfo(
        curve: Curve, s: Double, point: CurvePointInfo
    ): Derivatives = object : Derivatives {
        override val value: Double get() = point.tanAngle
        override val valueDeriv: Double get() = point.tanAngleDeriv
        override val valueSecondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle plus [angleOffset]. */
class OffsetTangentHeading(private val angleOffset: Double) : HeadingProvider {
    override fun getHeading(curve: Curve, s: Double): Double = curve.tanAngle(s) + angleOffset

    override fun getHeadingDeriv(curve: Curve, s: Double): Double = curve.tanAngleDeriv(s)

    override fun getHeadingSecondDeriv(curve: Curve, s: Double): Double = curve.tanAngleSecondDeriv(s)

    override fun getHeadingInfo(
        curve: Curve, s: Double, point: CurvePointInfo
    ): Derivatives = object : Derivatives {
        override val value: Double get() = point.tanAngle + angleOffset
        override val valueDeriv: Double get() = point.tanAngleDeriv
        override val valueSecondDeriv: Double get() = point.tanAngleSecondDeriv
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function from the range [0,1] for progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: MathFunction) : HeadingProvider {
    override fun getHeading(curve: Curve, s: Double): Double {
        return function(s / curve.length)
    }

    override fun getHeadingDeriv(curve: Curve, s: Double): Double {
        return function.deriv(s / curve.length)
    }

    override fun getHeadingSecondDeriv(curve: Curve, s: Double): Double {
        return function.secondDeriv(s / curve.length)
    }

    override fun getHeadingInfo(curve: Curve, s: Double, point: CurvePointInfo) = object : Derivatives {
        private val t = s / curve.length
        override val value: Double
            get() = function(s / curve.length)
        override val valueDeriv: Double
            get() = function.deriv(t)
        override val valueSecondDeriv: Double
            get() = function.secondDeriv(t)
    }
}

abstract class SplineInterpolatedHeading : HeadingProvider {
    init {
        TODO()
    }
}
///**
// * A [HeadingProvider] that smoothly interpolates between two different headings, including first derivatives, using a
// * cubic function
// */
//class SplineHeading(startAngle:Double, )