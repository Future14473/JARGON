package org.futurerobotics.jargon.math.function

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.calcCurvature
import org.futurerobotics.jargon.math.calcCurvatureDeriv

/**
 * Represents a Function in math with first, second, and third derivatives.
 */
interface MathFunction {

    /** The function's output at [t] */
    operator fun invoke(t: Double): Double

    /** The function's first derivative at [t] */
    fun deriv(t: Double): Double

    /** The function's second derivative at [t] */
    fun secondDeriv(t: Double): Double

    /** The function's third derivative at [t] */
    fun thirdDeriv(t: Double): Double
}

/**
 *  Represents a vector-valued function with first, second, and third derivatives.
 *  Includes curvature (with default implementations) too.
 */
interface VectorFunction {

    /** The function's vector output at [t] */
    fun vec(t: Double): Vector2d

    /** The function's vector derivative at [t] */
    fun vecDeriv(t: Double): Vector2d

    /** The function's second derivative at [t] */
    fun vecSecondDeriv(t: Double): Vector2d

    /** The function's third derivative at [t] */
    fun vecThirdDeriv(t: Double): Vector2d

    /** The function's curvature at [t] */
    fun curvature(t: Double): Double = calcCurvature(vecDeriv(t), vecSecondDeriv(t))

    /** The function's [curvature]'s derivative w/ respect to t, at [t] */
    fun curvatureDeriv(t: Double): Double = calcCurvatureDeriv(vecDeriv(t), vecSecondDeriv(t), vecThirdDeriv(t))
}

/**
 * A vector function defined by separate [x] and [y] component functions.
 *
 * @property x the x component function
 * @property y the y component function
 */
open class ComponentVectorFunction(protected val x: MathFunction, protected val y: MathFunction) : VectorFunction {
    override fun vec(t: Double): Vector2d = Vector2d(x(t), y(t))
    override fun vecDeriv(t: Double): Vector2d = Vector2d(x.deriv(t), y.deriv(t))
    override fun vecSecondDeriv(t: Double): Vector2d = Vector2d(x.secondDeriv(t), y.secondDeriv(t))
    override fun vecThirdDeriv(t: Double): Vector2d = Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))
    override fun toString(): String = "ComponentVecFunc(x: $x, y: $y)"
}
