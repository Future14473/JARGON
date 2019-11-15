package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.ValueMotionState
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.function.QuinticPolynomial
import org.futurerobotics.jargon.math.function.RealFunction
import org.futurerobotics.jargon.math.function.invoke
import org.futurerobotics.jargon.math.function.motionState

/** A [HeadingProvider] that maintains a constant `heading` */
class ConstantHeading(heading: Double) : HeadingProvider {

    private val output = ValueMotionState(angleNorm(heading), 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = output

    companion object {
        private const val serialVersionUID: Long = 6008569752536122191
    }
}

/**
 * A [HeadingProvider] that interpolates linearly along the curve from `fromHeading` to `toHeading`.
 * @see PolyInterpolatedHeading
 */
class LinearInterpolatedHeading(fromHeading: Double, toHeading: Double) : HeadingProvider {

    private val fromHeading = angleNorm(fromHeading)
    private val turnHeading = angleNorm(toHeading - fromHeading)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = turnHeading / point.length
        return ValueMotionState(angleNorm(fromHeading + s * dcl), dcl, 0.0)
    }

    companion object {
        private const val serialVersionUID: Long = -7150310678618833605
    }
}

/** A [HeadingProvider] that interpolates using a [QuinticPolynomial] along the curve from `fromHeading` to `toHeading`.
 *
 * This allows for smooth interpolation between curves.
 * @see LinearInterpolatedHeading
 */
class PolyInterpolatedHeading(fromHeading: MotionState<Double>, toHeading: MotionState<Double>) : HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromHeading, toHeading)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = spline.motionState(s / point.length)

    companion object {
        private const val serialVersionUID: Long = 207160958359032108
    }
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle, plus [offset]. */
open class TangentHeading
@JvmOverloads constructor(offset: Double = 0.0) : HeadingProvider {

    private val offset = angleNorm(offset)
    final override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = object : MotionState<Double> {
        override val value: Double get() = angleNorm(point.tanAngle + offset)
        override val deriv: Double get() = point.tanAngleDeriv
        override val secondDeriv: Double get() = point.tanAngleSecondDeriv
    }

    /**
     * The default tangent heading with angle offset of 0.
     */
    companion object Default : TangentHeading(0.0) {

        private const val serialVersionUID: Long = 5331301765766845360
    }
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is polynomial interpolated from `fromOffset` to
 * `toOffset`
 */
class PolyInterpolatedTangentHeading(fromOffset: MotionState<Double>, toOffset: MotionState<Double>) : HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromOffset, toOffset)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val state = spline.motionState(s / point.length)
        return object : MotionState<Double> {
            override val value: Double get() = angleNorm(point.tanAngle + state.value)
            override val deriv: Double get() = point.tanAngleDeriv + state.deriv
            override val secondDeriv: Double get() = point.tanAngleSecondDeriv + state.secondDeriv
        }
    }

    companion object {
        private const val serialVersionUID: Long = -4119579837615657805
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: RealFunction) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = object : MotionState<Double> {
        private val t = s / point.length
        override val value: Double get() = angleNorm(function(t))
        override val deriv: Double get() = function.deriv(t)
        override val secondDeriv: Double get() = function.secondDeriv(t)
    }

    companion object {
        private const val serialVersionUID: Long = -9087053844463217786
    }
}
