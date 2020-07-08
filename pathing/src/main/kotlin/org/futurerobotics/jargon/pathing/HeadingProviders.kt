package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.AnyMotionState
import org.futurerobotics.jargon.math.RealFunction
import org.futurerobotics.jargon.math.RealMotionState
import org.futurerobotics.jargon.math.angleNorm

/** A [HeadingProvider] that maintains a constant heading. */
class ConstantHeading(heading: Double) : HeadingProvider {

    private val output = RealMotionState(angleNorm(heading), 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): RealMotionState = output
}

/**
 * A [HeadingProvider] that interpolates linearly along the curve from `fromHeading` to `toHeading`.
 * @see PolyInterpHeading
 */
class LinInterpHeading(fromHeading: Double, toHeading: Double) : HeadingProvider {

    private val fromHeading = angleNorm(fromHeading)
    private val turnHeading = angleNorm(toHeading - fromHeading)

    override fun getHeading(point: CurvePoint, s: Double): RealMotionState {
        val dcl = turnHeading / point.curveLength
        return RealMotionState(angleNorm(fromHeading + s * dcl), dcl, 0.0)
    }
}

/**
 * A [HeadingProvider] that interpolates using a [QuinticPolynomial] along the curve from `fromHeading` to `toHeading`.
 *
 * This allows for smooth interpolation between curves.
 * @see LinInterpHeading
 */
class PolyInterpHeading(fromHeading: AnyMotionState<Double>, toHeading: AnyMotionState<Double>) :
    HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromHeading, toHeading)
    override fun getHeading(point: CurvePoint, s: Double): RealMotionState =
        spline.motionState(s / point.curveLength)
}

/**
 *  A [HeadingProvider] that has the heading equal to the curve's tangent angle, plus an optional [offset].
 *
 * The companion object a [TangentHeading] with 0 offset.
 */
open class TangentHeading
@JvmOverloads constructor(offset: Double = 0.0) : HeadingProvider {

    private val offset = angleNorm(offset)
    final override fun getHeading(point: CurvePoint, s: Double): RealMotionState =
        RealMotionState(
            angleNorm(point.tanAngle + offset),
            point.tanAngleDeriv,
            point.tanAngleSecondDeriv
        )

    companion object : TangentHeading(0.0)
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is linearly interpolated from `fromOffset` to
 * `toOffset`.
 *
 * @see PolyInterpTangentHeading
 */
open class LinInterpTangentHeading
constructor(fromOffset: Double, toOffset: Double) : HeadingProvider {

    private val fromOffset = angleNorm(fromOffset)
    private val turnOffset = angleNorm(toOffset - this.fromOffset)

    override fun getHeading(point: CurvePoint, s: Double): RealMotionState {
        val offsetPerLength = turnOffset / point.curveLength
        return RealMotionState(
            angleNorm(point.tanAngle + fromOffset + s * offsetPerLength),
            offsetPerLength + point.tanAngle,
            point.tanAngleSecondDeriv
        )
    }
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is polynomial interpolated from `fromOffset` to
 * `toOffset`.
 *
 * @see LinInterpTangentHeading
 */
class PolyInterpTangentHeading
constructor(fromOffset: AnyMotionState<Double>, toOffset: AnyMotionState<Double>) : HeadingProvider {

    private val spline = QuinticPolynomial.fromDerivatives(fromOffset, toOffset)

    override fun getHeading(point: CurvePoint, s: Double): RealMotionState {
        val state = spline.motionState(s / point.curveLength)
        return RealMotionState(
            angleNorm(point.tanAngle + state.s),
            point.tanAngleDeriv + state.v,
            point.tanAngleSecondDeriv + state.a
        )
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: RealFunction) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): RealMotionState {
        val t = s / point.curveLength
        return RealMotionState(
            angleNorm(function.value(t)),
            function.deriv(t),
            function.secondDeriv(t)
        )
    }
}
