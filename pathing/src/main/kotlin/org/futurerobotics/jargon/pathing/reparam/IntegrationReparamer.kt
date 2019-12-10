package org.futurerobotics.jargon.pathing.reparam

import org.futurerobotics.jargon.math.function.VectorFunction
import org.futurerobotics.jargon.pathing.reparam.IntegrationReparamer.reparam

/**
 * Reparameterization by numerical integration using midpoint Riemann sums.
 * The function that actually does this is [reparam].
 */
object IntegrationReparamer {

    /** The default stepsPerSample used in [IntegrationReparamer]'s overloads/default parameters. */
    const val defaultStepsPerSample: Int = 4
    /** The default numSamples used for [IntegrationReparamer] for overloads/default parameters. */
    const val defaultNumSamples: Int = 250

    /**
     * Reparameterizes a [VectorFunction] [func] by using a midpoint Riemann sum integration.
     * This will result in a [ReparamCurve] with [numSamples] of evenly spaced samples along the function's parameter,
     * and each sample will use [stepsPerSample] subdivisions in the Riemann sum integration.
     */
    @JvmStatic
    @JvmOverloads
    fun reparam(
        func: VectorFunction, numSamples: Int = defaultNumSamples, stepsPerSample: Int = defaultStepsPerSample
    ): ReparamCurve {
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
            s += func.vecDeriv(tMid).length / totalSteps
            if (step % stepsPerSample == 0) {
                val t = step.toDouble() / totalSteps
                sSamples[i] = s
                tSamples[i] = t
                i++
            }
        }
        return ReparamCurve(func, SamplesReparamMapping.fromPointSamples(sSamples, tSamples))
    }
}

/** Convenience extension method for [IntegrationReparamer.reparam] */
fun VectorFunction.reparamByIntegration(
    numSamples: Int = IntegrationReparamer.defaultNumSamples,
    stepsPerSample: Int = IntegrationReparamer.defaultStepsPerSample
): ReparamCurve = reparam(
    this, numSamples, stepsPerSample
)
