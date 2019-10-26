package org.futurerobotics.jargon.pathing.reparam

import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.util.Steppable
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.replaceIf

/**
 * Represents a mapping of s values (arc length) to t value (parameter on a parametric func).
 */
interface ReparamMapping : Steppable<Double, Double> {

    /** The total length of this mapping; i.e., the last sample's `s` value. */
    val length: Double

    /** Returns the t on the original parametric function associated with [s] units along the curve. */
    fun tOfS(s: Double): Double

    /** Returns a stepper for [tOfS] */
    @JvmDefault
    override fun stepper(): Stepper<Double, Double> = Stepper(::tOfS)
}

/**
 * A [ReparamMapping] using linearly interpolated samples of s to t values.
 *
 * See factory methods.
 */
class SamplesReparamMapping
private constructor(
    private val sSamples: DoubleArray, private val tSamples: DoubleArray
) : ReparamMapping {

    /** The total length of this mapping; i.e., the last sample's `s` value. */
    override val length: Double get() = sSamples.last()

    /** The total number of samples. */
    val numSamples: Int get() = sSamples.size

    init {
        require(sSamples.size == tSamples.size)
        { "sSamples's size (${sSamples.size}) and tSamples's size (${tSamples.size}) need to match." }
        require(numSamples >= 2)
        { "Need at least 2 samples, got $numSamples instead" }
        require(sSamples.first() epsEq 0.0 && tSamples.first() epsEq 0.0)
        { "First sample's s (${sSamples.first()}) must be 0 and t (${tSamples.first()}) must epsEq 0" }
        require(tSamples.last() epsEq 1.0)
        { "Last sample's t (${tSamples.last()}) should epsEq 1.0" }
        var prevS = sSamples.first()
        var prevT = tSamples.first()
        for (i in 1 until numSamples) {
            val curS = sSamples[i]
            val curT = tSamples[i]

            require(curS > prevS) { "Samples not sorted on s, got previous s $prevS and after s $curS" }
            require(curT > prevT) { "Samples not sorted on t, got previous t $prevT and after t $curT" }

            prevS = curS
            prevT = curT
        }
    }

    override fun tOfS(s: Double): Double {
        var i = sSamples.binarySearch(s)
        if (i >= 0) return tSamples[i]
        i = (-i - 2).coerceIn(0, sSamples.lastIndex - 1)
        return getAtSeg(i, s)
    }

    private fun getAtSeg(index: Int, s: Double): Double {
        val sBefore = sSamples[index]
        val tBefore = tSamples[index]
        val sAfter = sSamples[index + 1]
        val tAfter = tSamples[index + 1]
        val progress = (s - sBefore) / (sAfter - sBefore)
        return tBefore + progress * (tAfter - tBefore)
    }

    override fun stepper(): Stepper<Double, Double> = object : Stepper<Double, Double> {
        private var i = -1

        override fun stepTo(step: Double): Double {
            //Compare me to SegmentsMotionProfile; difference since max index is actual lastIndex - 1
            if (i == -1) {
                i = when {
                    step <= 0 -> 0
                    step >= length -> sSamples.lastIndex - 1
                    else -> sSamples.binarySearch(step)
                        .replaceIf({ it < 0 }) { -it - 2 }
                        .coerceIn(0, sSamples.lastIndex - 1)
                }
            } else {
                while (i < sSamples.lastIndex - 1 && step >= sSamples[i + 1]) i++
                while (i > 0 && step < sSamples[i]) i--
            }
            return getAtSeg(i, step)
        }
    }

    companion object {
        /**
         * Creates a reparam mapping from a list of (s,t) samples.
         */
        @JvmStatic
        fun fromSTPairs(samples: List<Pair<Double, Double>>): SamplesReparamMapping {
            val size = samples.size
            val sSamples = DoubleArray(size)
            val tSamples = DoubleArray(size)
            samples.forEachIndexed { i, (first, second) ->
                sSamples[i] = first
                tSamples[i] = second
            }
            return SamplesReparamMapping(sSamples, tSamples)
        }

        /**
         * Creates a reparam mapping from a list of sSamples and corresponding tSamples.
         */
        @JvmStatic
        fun fromPointSamples(sSamples: List<Double>, tSamples: List<Double>): SamplesReparamMapping {
            return SamplesReparamMapping(
                sSamples.toDoubleArray(),
                tSamples.toDoubleArray()
            )
        }

        /**
         * Creates a reparam mapping from a list of sSamples and corresponding tSamples.
         */
        @JvmStatic
        fun fromPointSamples(sSamples: DoubleArray, tSamples: DoubleArray): SamplesReparamMapping {
            return SamplesReparamMapping(
                sSamples.copyOf(),
                tSamples.copyOf()
            )
        }
    }
}
