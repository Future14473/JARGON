package org.futurerobotics.jargon.pathing.reparam

import org.futurerobotics.jargon.math.VectorFunction

/**
 * A [Reparameterizer] that uses numerical integration using midpoint Riemann sums.
 *
 * @param numSamples the total number of samples to in the resulting [ReparamMapping]
 * @param stepsPerSample the total number of integration steps to take per sample.
 */
class IntegrationReparameterizer(
    val numSamples: Int = 250,
    val stepsPerSample: Int = 4
) : Reparameterizer {

    override fun reparameterize(function: VectorFunction): ReparamMapping {
        var s = 0.0
        val totalStepsLong = ((numSamples - 1).toLong() * stepsPerSample)
        require(totalStepsLong <= Int.MAX_VALUE) { "Too many total steps!!!" }
        val totalSteps = totalStepsLong.toInt()
        val sSamples = DoubleArray(numSamples)
        val tSamples = DoubleArray(numSamples)
        sSamples[0] = 0.0
        tSamples[0] = 0.0
        var i = 1
        repeat(totalSteps) {
            val step = it + 1
            val tMid = (it + 0.5) / totalSteps
            s += function.deriv(tMid).length / totalSteps
            if (step % stepsPerSample == 0) {
                val t = step.toDouble() / totalSteps
                sSamples[i] = s
                tSamples[i] = t
                i++
            }

        }
        return SamplesReparamMapping(sSamples, tSamples)
    }
}
