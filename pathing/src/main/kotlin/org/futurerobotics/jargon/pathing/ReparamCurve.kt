package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.*
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.replaceIf

/**
 * A [Curve] that works by reparameterizing an arbitrary continuous [VectorFunction] ([func]), using a
 * [ReparamMapping] ([mapping]) that maps arc length to the original function parameter.
 *
 * If you want to create your own re-parameterization, make a implementation [ReparamMapping].
 */
class ReparamCurve(internal val func: VectorFunction, internal val mapping: ReparamMapping) : Curve {

    constructor(function: VectorFunction, reparameterizer: Reparameterizer) :
            this(function, reparameterizer.reparameterize(function))

    override val length: Double get() = mapping.length

    override fun pointAt(s: Double): CurvePoint = Point(mapping.tOfS(s))

    override fun stepper(): Stepper<CurvePoint> {
        val mappingStepper = mapping.stepper()
        return Stepper { s ->
            Point(mappingStepper.stepTo(s))
        }
    }

    internal inner class Point(t: Double) : CurvePoint {
        private val p: Vector2d = func.value(t)
        private val v: Vector2d = func.deriv(t)
        private val a: Vector2d = func.secondDeriv(t)
        private val j: Vector2d = func.thirdDeriv(t)
        override val originalLength: Double get() = this@ReparamCurve.length
        override val position: Vector2d get() = p
        private var _positionDeriv: Vector2d? = null
        override val positionDeriv: Vector2d
            get() = _positionDeriv ?: v.normalized()
                .replaceIf({ it.isNaN() }) { Vector2d.ZERO }
                .also { _positionDeriv = it }
        override val positionSecondDeriv: Vector2d
            get() = tanAngleDeriv zcross positionDeriv
        override val tanAngle: Double
            get() = v.angle
        private var _tanAngleDeriv = Double.NaN
        override val tanAngleDeriv: Double
            get() = _tanAngleDeriv.ifNan {
                (v cross a / v.lengthPow(3.0))
                    .ifNan { 0.0 }
                    .also { _tanAngleDeriv = it }
            }
        private var _tanAngleSecondDeriv = Double.NaN
        override val tanAngleSecondDeriv: Double
            get() = _tanAngleSecondDeriv.ifNan {
                ((v cross j) / v.lengthPow(4.0) - 3 * tanAngleDeriv * (v dot a) / v.lengthPow(3.0))
                    .ifNan { 0.0 }
                    .also { _tanAngleSecondDeriv = it }
            }

        internal fun motionState() = MotionState(p, v, a)
    }
}