package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.function.RealFunction

/** A [HeadingProvider] that maintains a constant heading of [angle] */
class ConstantHeading(angle: Double) : HeadingProvider {

    private val angle = angleNorm(angle)
    private val derivatives = ValueDerivatives(angle, 0.0, 0.0)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = derivatives

    companion object {
        private const val serialVersionUID = 6008569752536122191
    }
}

/** A [HeadingProvider] that interpolates linearly among two angles, starting at [fromAngle] and turning [turnAngle] */
class LinearInterpolatedHeading(private val fromAngle: Double, private val turnAngle: Double) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> {
        val dcl = turnAngle / point.length
        return ValueDerivatives(angleNorm(fromAngle + s * dcl), dcl, 0.0)
    }

    companion object {
        private const val serialVersionUID = -7150310678618833605
    }
}

/** The singleton [HeadingProvider] that has the heading equal to the curve's tangent angle. Use for non-holonomic drives. */
object TangentHeading : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = object :
        MotionState<Double> {
        override val value: Double get() = point.tanAngle
        override val vel: Double get() = point.tanAngleDeriv
        override val accel: Double get() = point.tanAngleSecondDeriv
    }

    private const val serialVersionUID = 2553159564085724309

    private fun readResolve(): Any = this
}

/** A [HeadingProvider] that has the heading equal to the curve's tangent angle plus [angleOffset]. */
class OffsetTangentHeading(angleOffset: Double) : HeadingProvider {

    private val angleOffset = angleNorm(angleOffset)
    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = object :
        MotionState<Double> {
        override val value: Double get() = angleNorm(point.tanAngle + angleOffset)
        override val vel: Double get() = point.tanAngleDeriv
        override val accel: Double get() = point.tanAngleSecondDeriv
    }

    companion object {
        private const val serialVersionUID = 5331301765766845360
    }
}

/**
 * A [HeadingProvider] that uses an arbitrary function on the domain [0, 1] corresponding to progress along a curve
 * to provide heading.
 */
class FunctionHeading(private val function: RealFunction) : HeadingProvider {

    override fun getHeading(point: CurvePoint, s: Double): MotionState<Double> = object :
        MotionState<Double> {
        private val t = s / point.length
        override val value: Double
            get() = angleNorm(function(s / point.length))
        override val vel: Double
            get() = function.deriv(t)
        override val accel: Double
            get() = function.secondDeriv(t)
    }

    companion object {
        private const val serialVersionUID = -9087053844463217786
    }
}
