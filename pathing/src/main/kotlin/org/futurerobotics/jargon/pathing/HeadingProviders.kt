package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.ValueMotionState
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.function.RealFunction
import org.futurerobotics.jargon.math.function.invoke

/** A [HeadingProvider] that maintains a constant heading of [angle] */
class ConstantHeading(heading: Double) : HeadingProvider {

    private val angle = angleNorm(heading)
    private val motionState = ValueMotionState(heading, 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = motionState

    companion object {
        private const val serialVersionUID = 6008569752536122191
    }
}

/** A [HeadingProvider] that interpolates linearly among two angles, starting at [fromAngle] and turning [turnAngle] */
class LinearInterpolatedHeading(fromHeading: Double, private val turnAngle: Double) : HeadingProvider {

    private val fromAngle = angleNorm(fromHeading)

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = turnAngle / point.length
        return ValueMotionState(angleNorm(fromAngle + s * dcl), dcl, 0.0)
    }

    companion object {
        private const val serialVersionUID = -7150310678618833605
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

        private const val serialVersionUID = 5331301765766845360
    }
}

/**
 * A [HeadingProvider] that is like [TangentHeading], but the offset is linearly interpolated from `fromOffset` to
 * [offsetTurnAngle].
 */
class LerpTangentHeading(fromOffset: Double, private val offsetTurnAngle: Double) : HeadingProvider {

    private val fromOffset = angleNorm(fromOffset)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = offsetTurnAngle / point.length
        return object : MotionState<Double> {
            override val value: Double get() = angleNorm(point.tanAngle + fromOffset + s * dcl)
            override val deriv: Double get() = point.tanAngleDeriv + dcl
            override val secondDeriv: Double get() = point.tanAngleSecondDeriv
        }
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
        private const val serialVersionUID = -9087053844463217786
    }
}
