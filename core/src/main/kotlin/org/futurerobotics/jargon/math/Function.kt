package org.futurerobotics.jargon.math

/**
 * Represents a real valued function, with first, second, and third derivatives.
 */
interface RealFunction {

    /** The function's value at [t] */
    fun value(t: Double): Double

    /** The function's first derivative at [t] */
    fun deriv(t: Double): Double

    /** The function's second derivative at [t] */
    fun secondDeriv(t: Double): Double

    /** The function's third derivative at [t] */
    fun thirdDeriv(t: Double): Double

    /**
     * Gets a [RealMotionState] for the value and first and second derivatives at [t].
     */
    fun motionState(t: Double): RealMotionState = RealMotionState(value(t), deriv(t), secondDeriv(t))
}

/** [RealFunction.value] */
operator fun RealFunction.invoke(t: Double): Double = value(t)

/**
 * Represents a vector-valued function with first, second, and third derivatives.
 *
 * Includes curvature in default implementations too.
 */
interface VectorFunction {

    /** The function's vector value at [t]. */
    fun value(t: Double): Vector2d

    /** The function's vector derivative at [t]. */
    fun deriv(t: Double): Vector2d

    /** The function's second derivative at [t]. */
    fun secondDeriv(t: Double): Vector2d

    /** The function's third derivative at [t]. */
    fun thirdDeriv(t: Double): Vector2d

    /** The function's curvature at [t] */
    fun curvature(t: Double): Double {
        val v = deriv(t)
        return (v cross secondDeriv(t)) / v.lengthPow(3.0)
    }

    /** The function's [curvature]'s derivative w/ respect to t, at [t] */
    fun curvatureDeriv(t: Double): Double {
        val v = deriv(t)
        val a = secondDeriv(t)
        val j = thirdDeriv(t)
        return (v cross j) / v.lengthPow(3.0) - 3 * (v cross a) * (v dot a) / v.lengthPow(5.0)
    }

    /** Gets a [MotionState] for the value and first and second derivatives at [t]. */
    fun motionState(t: Double): MotionState<Vector2d> =
        MotionState(value(t), deriv(t), secondDeriv(t))
}

/** [VectorFunction.value] */
operator fun VectorFunction.invoke(t: Double): Vector2d = value(t)

/**
 * A vector function defined by separate [x] and [y] component functions.
 *
 * @property x the x function
 * @property y the y function
 */
open class ComponentVectorFunction(protected val x: RealFunction, protected val y: RealFunction) : VectorFunction {

    override fun value(t: Double): Vector2d = Vector2d(x(t), y(t))
    override fun deriv(t: Double): Vector2d = Vector2d(x.deriv(t), y.deriv(t))
    override fun secondDeriv(t: Double): Vector2d = Vector2d(x.secondDeriv(t), y.secondDeriv(t))
    override fun thirdDeriv(t: Double): Vector2d = Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))
    override fun toString(): String = "ComponentVecFunc(x: $x, y: $y)"
}
