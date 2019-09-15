package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Interval

/**
 * PID coefficients with more options like some regulation on that pesky [i] term.
 *
 * @param p the proportional gain
 * @param i the integral gain
 * @param d the derivative gain
 * @param errorBounds the bounds on the calculated error; used to avoid wild p-terms
 * @param outputBounds the bounds on the output; used to avoid wild outputs.
 * @param integralActivationThreshold The maximum error that the integral term can activate on.
 * @param maxIntegralContribution The maximum contribution to the output the integral term can have.
 */
open class PIDCoefficients(
    val p: Double,
    val i: Double,
    val d: Double,
    val errorBounds: Interval = Interval.REAL,
    val outputBounds: Interval = Interval.REAL,
    val integralActivationThreshold: Double = Double.POSITIVE_INFINITY,
    val maxIntegralContribution: Double = Double.POSITIVE_INFINITY
) {
    init {
        require(p >= 0) { "p term ($p) must be >= 0" }
        require(i >= 0) { "p term ($i) must be >= 0" }
        require(d >= 0) { "p term ($d) must be >= 0" }
        require(errorBounds.isNotEmpty()) { "errorBounds must not be empty" }
        require(outputBounds.isNotEmpty()) { "errorBounds must not be empty" }
        require(integralActivationThreshold >= 0)
        { "integralActivationThreshold ($integralActivationThreshold) must be >= 0" }
        require(maxIntegralContribution >= 0)
        { " maxIntegralContribution ($ maxIntegralContribution) must be >= 0" }
    }

    /**
     * The maximum Error sum due to `maxIntegralContribution` (see [PIDFCoefficients])
     */
    val maxErrorSum: Double = maxIntegralContribution / i
}

/**
 * PIDF coefficients, _with feed forward on velocity and acceleration_,
 * with more optional options like some regulation on that pesky [i] term.
 *
 * @see [PIDCoefficients]
 * @param p the proportional gain
 * @param i the integral gain
 * @param d the derivative gain
 * @param fv the feed-forward gain for velocity
 * @param fa the feed-forward gain for acceleration
 * @param errorBounds the bounds on the calculated error; used to avoid wild p-terms
 * @param outputBounds the bounds on the output; used to avoid wild outputs.
 * @param integralActivationThreshold The maximum error that the integral term can activate on.
 * @param maxIntegralContribution The maximum contribution to the output the integral term can have.
 */
class PIDFCoefficients(
    p: Double,
    i: Double,
    d: Double,
    val fv: Double = 0.0,
    val fa: Double = 0.0,
    errorBounds: Interval = Interval.REAL,
    outputBounds: Interval = Interval.REAL,
    integralActivationThreshold: Double = Double.POSITIVE_INFINITY,
    maxIntegralContribution: Double = Double.POSITIVE_INFINITY
) : PIDCoefficients(p, i, d, errorBounds, outputBounds, integralActivationThreshold, maxIntegralContribution) {
    init {
        require(fv >= 0) { "fv term ($fv) must be >= 0" }
        require(fa >= 0) { "fa term ($fa) must be >= 0" }
    }
}