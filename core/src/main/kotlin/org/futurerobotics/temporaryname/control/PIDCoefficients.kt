package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.Interval

/**
 * PID coefficients with more optional options like some regulation on that pesky [i] term.
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
    maxIntegralContribution: Double = Double.POSITIVE_INFINITY
) {

    /**
     * The maximum integral range due to `maxIntegralContribution` (see [PIDFCoefficients] constructor)
     */
    val maxErrorSum: Double = maxIntegralContribution / i
}

/**
 * PID coefficients, _with feed forward on velocity and acceleration_,
 * with more optional options like some regulation on that pesky [i] term.
 *
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
) : PIDCoefficients(p, i, d, errorBounds, outputBounds, integralActivationThreshold, maxIntegralContribution)