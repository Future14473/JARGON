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
}

/**
 * A [HeadingProvider] that interpolates linearly along the curve from `fromHeading` to `toHeading`.
 * @see PolyInterpolatedHeading
 */
class LinearlyInterpolatedHeading(fromHeading: Double, toHeading: Double) : HeadingProvider {

    private val fromHeading = angleNorm(fromHeading)
    private val turnHeading = angleNorm(toHeading - fromHeading)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = turnHeading / point.length
        return ValueMotionState(angleNorm(fromHeading + s * dcl), dcl, 0.0)
    }
}

/** A [HeadingProvider] that interpolates using a [QuinticPolynomial] along the curve from `fromHeading` to `toHeading`.
 *
 * This allows for smooth interpolation between curves.
 * @see LinearlyInterpolatedHeading
 */
class PolyInterpolatedHeading(fromHeading: MotionState<Double>, toHeading: MotionState<Double>) : HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromHeading, toHeading)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = spline.motionState(s / point.length)
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle, plus [offset]. */
open class TangentHeading
@JvmOverloads constructor(offset: Double = 0.0) : HeadingProvider {

    private val offset = angleNorm(offset)
    final override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> =
        ValueMotionState(
            angleNorm(point.tanAngle + offset),
            point.tanAngleDeriv,
            point.tanAngleSecondDeriv
        )

    companion object : TangentHeading(0.0)
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is linearly interpolated from `fromOffset` to
 * `toOffset`.
 */
open class LinearlyInterpolatedTangentHeading
constructor(fromOffset: Double, toOffset: Double) : HeadingProvider {

    private val fromOffset = angleNorm(fromOffset)
    private val turnOffset = angleNorm(toOffset - this.fromOffset)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = turnOffset / point.length
        return ValueMotionState(
            angleNorm(point.tanAngle + fromOffset + s * dcl),
            dcl + point.tanAngle,
            point.tanAngleSecondDeriv
        )
    }
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is polynomial interpolated from `fromOffset` to
 * `toOffset`.
 */
class PolyInterpolatedTangentHeading(fromOffset: MotionState<Double>, toOffset: MotionState<Double>) : HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromOffset, toOffset)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val state = spline.motionState(s / point.length)
        return ValueMotionState(
            angleNorm(point.tanAngle + state.value),
            point.tanAngleDeriv + state.deriv,
            point.tanAngleSecondDeriv + state.secondDeriv
        )
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: RealFunction) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val t = s / point.length
        return ValueMotionState(
            angleNorm(function(t)),
            function.deriv(t),
            function.secondDeriv(t)
        )
    }
}
