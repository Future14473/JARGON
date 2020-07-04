package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.math.Interval

/**
 * Simple PID coefficients.
 *
 * @param p the proportional gain
 * @param i the integral gain
 * @param d the derivative gain
 * @see [PIDFCoefficients]
 */
open class PIDCoefficients(@JvmField val p: Double, @JvmField val i: Double, @JvmField val d: Double)
//
///**
// * Simple PIDF coefficients. The feed forward (F) is static.
// *
// * @param p the proportional gain
// * @param i the integral gain
// * @param d the derivative gain
// * @param f the feed-forward.
// */
//open class PIDFCoefficients(p: Double, i: Double, d: Double, @JvmField val f: Double) : PIDCoefficients(p, i, d)

/**
 * PID coefficients with more options with regulation on the pesky [i] term.
 *
 * @param p the proportional gain
 * @param i the integral gain
 * @param d the derivative gain
 * @param errorBounds bounds on the calculated error; used to avoid wild p-terms
 * @param outputBounds bounds on the output; used to avoid wild outputs.
 * @param integralActivationThreshold The maximum error that the integral term can activate on.
 * @param maxIntegralContribution The maximum contribution to the output the integral term can have.
 */
class ExtendedPIDCoefficients
@JvmOverloads constructor(
    p: Double,
    i: Double,
    d: Double,
    val errorBounds: Interval = Interval.REAL,
    val outputBounds: Interval = Interval.REAL,
    val integralActivationThreshold: Double = Double.POSITIVE_INFINITY,
    maxIntegralContribution: Double = Double.POSITIVE_INFINITY
) : PIDCoefficients(p, i, d) {

    init {
        require(p >= 0) { "p term ($p) must be >= 0" }
        require(i >= 0) { "p term ($i) must be >= 0" }
        require(d >= 0) { "p term ($d) must be >= 0" }
        require(errorBounds.isNotEmpty()) { "errorBounds must not be empty" }
        require(outputBounds.isNotEmpty()) { "errorBounds must not be empty" }
        require(integralActivationThreshold >= 0) { "integralActivationThreshold ($integralActivationThreshold) must be >= 0" }
        require(maxIntegralContribution >= 0) { " maxIntegralContribution ($ maxIntegralContribution) must be >= 0" }
    }

    /**
     * The maximum Error sum due to `maxIntegralContribution` (see [PIDCoefficients])
     */
    val maxErrorSum: Double = maxIntegralContribution / i
}
