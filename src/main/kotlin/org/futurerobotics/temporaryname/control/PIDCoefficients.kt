package org.futurerobotics.temporaryname.control

import org.futurerobotics.temporaryname.math.EPSILON
import org.futurerobotics.temporaryname.math.Interval
import org.futurerobotics.temporaryname.util.replaceIf

/**
 * PID coefficients with more options,
 * Like some regulation on that pesky integral term ([integralActivationThreshold], [integralActivationThreshold])
 *
 * @property p the proportional gain
 * @property i the integral gain
 * @property d the derivative gain
 * @property inputBounds the bounds on the input; used to avoid wild p-terms
 * @property outputBounds the bounds on the output; used to avoid wild outputs. Sometimes you don't want to use this...
 * @property integralActivationThreshold The maximum error that the integral term can activate on.
 * @param maxIntegralContribution The maximum contribution to the output the integral term can have.
 */
class PIDCoefficients(
    val p: Double,
    val i: Double,
    val d: Double,
    val inputBounds: Interval = Interval.REAL,
    val outputBounds: Interval = Interval.REAL,
    val integralActivationThreshold: Double = Double.POSITIVE_INFINITY,
    maxIntegralContribution: Double = Double.POSITIVE_INFINITY
) {
    /**
     * The maximum integral range due to `maxIntegralContribution` (see [PIDCoefficients] constructor)
     */
    val maxErrorSum: Double = maxIntegralContribution / i.replaceIf({ it <= EPSILON }) { Double.POSITIVE_INFINITY }
}