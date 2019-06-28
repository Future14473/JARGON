package org.futurerobotics.temporaryname.math.function

import org.futurerobotics.temporaryname.math.Derivatives

/**
 * A Quintic Polynomial Math function, specified by coefficients,
 * in the form at^5+bt^4+ct^3+dt^2+et+f
 */
class QuinticPolynomial(
    @JvmField val a: Double, @JvmField val b: Double, @JvmField val c: Double, @JvmField val d: Double, @JvmField val e: Double, @JvmField val f: Double
) : MathFunction {
    override fun invoke(t: Double): Double {
        return ((((a * t + b) * t + c) * t + d) * t + e) * t + f
    }

    override fun deriv(t: Double): Double {
        return (((5 * a * t + 4 * b) * t + 3 * c) * t + 2 * d) * t + e
    }

    override fun secondDeriv(t: Double): Double {
        return ((20 * a * t + 12 * b) * t + 6 * c) * t + 2 * d
    }

    override fun thirdDeriv(t: Double): Double {
        return (60 * a * t + 24 * b) * t + 6 * c
    }

    override fun toString(): String {
        return "QuinticPoly(%.4ft^5+%.4ft^4+%.4ft^3+%.4ft^2+%.4ft+%.4f)".format(a, b, c, d, e, f)
    }

    companion object {
        internal fun fromControlPoints(
            p0: Double, p1: Double, p2: Double, p3: Double, p4: Double, p5: Double
        ): QuinticPolynomial {
            val a = -p0 + 5 * p1 - 10 * p2 + 10 * p3 - 5 * p4 + p5
            val b = 5 * p0 - 20 * p1 + 30 * p2 - 20 * p3 + 5 * p4
            val c = -10 * p0 + 30 * p1 - 30 * p2 + 10 * p3
            val d = 10 * p0 - 20 * p1 + 10 * p2
            val e = -5 * p0 + 5 * p1
            val f = p0
            return QuinticPolynomial(a, b, c, d, e, f)
        }

        /**
         * Creates a Quintic polynomial defined by the value and first and second derivatives of endpoints.
         */
        @JvmStatic
        fun fromDerivatives(
            start: Double,
            startDeriv: Double,
            startSecondDeriv: Double,
            end: Double,
            endDeriv: Double,
            endSecondDeriv: Double
        ): QuinticPolynomial {
//            val a = -s+5*(s+s_1/5)-10*(s_2/20+2*(s+s_1/5)-s)+10*(f_2/20+2*(f-f_1/5)-f)-5*(f-f_1/5)+f
//            val b = 5*s-20*(s+s_1/5)+30*(s_2/20+2*(s+s_1/5)-s)-20*(f_2/20+2*(f-f_1/5)-f)+5*(f-f_1/5)
//            val c = -10*s+30*(s+s_1/5)-30*(s_2/20+2*(s+s_1/5)-s)+10*(f_2/20+2*(f-f_1/5)-f)
            //equation solvers was used.
            val a = -6 * start - 3 * startDeriv - 0.5 * startSecondDeriv + 6 * end - 3 * endDeriv + 0.5 * endSecondDeriv
            val b = 15 * start + 8 * startDeriv + 1.5 * startSecondDeriv - 15 * end + 7 * endDeriv - endSecondDeriv
            val c =
                -10 * start - 6 * startDeriv - 1.5 * startSecondDeriv + 10 * end - 4 * endDeriv + 0.5 * endSecondDeriv
            val d = startSecondDeriv / 2
            val e = startDeriv
            val f = start
            return QuinticPolynomial(a, b, c, d, e, f)
        }

        /**
         * Creates a Quintic polynomial defined by then start and endpoint values and derivatives.
         */
        @JvmStatic
        fun fromDerivatives(
            start: Derivatives, end: Derivatives
        ): QuinticPolynomial = fromDerivatives(
            start.value, start.valueDeriv, start.valueSecondDeriv, end.value, end.valueDeriv, end.valueSecondDeriv
        )
    }
}

