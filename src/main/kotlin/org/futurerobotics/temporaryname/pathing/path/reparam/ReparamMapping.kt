package org.futurerobotics.temporaryname.pathing.path.reparam

import org.futurerobotics.temporaryname.math.epsEq
import org.futurerobotics.temporaryname.util.replaceIf


/**
 * Represents a linearly interpolated mapping of s values (arc length) to t value (parameter on a parametric func).
 */
class ReparamMapping private constructor(private val sSamples: DoubleArray, private val tSamples: DoubleArray) {
    /** The total number of samples. */
    val numSamples: Int = sSamples.size

    init {
        //        require(sSamples.size==tSamples.size)
        require(numSamples >= 2) { "Need at least 2 samples, got $numSamples instead" }
        require(sSamples.first() == 0.0 && tSamples.first() == 0.0) {
            "First sample's s (${sSamples.first()}) must be 0 and t (${tSamples.first()}) must be 0"
        }
        require(tSamples.last() epsEq 1.0) { "Last sample's t ${tSamples.last()} should epsEq 1.0" }
        var prevS = sSamples.first()
        var prevT = tSamples.first()
        for (i in 1 until numSamples) {
            require(sSamples[i] > prevS) { "Samples not sorted on s, got previous s $prevS and after s ${sSamples[i]}" }
            require(tSamples[i] > prevT) { "Samples not sorted on s, got previous t $prevT and after t ${tSamples[i]}" }

            prevS = sSamples[i]
            prevT = tSamples[i]
        }
    }


    /** The total length of this mapping; i.e. the last sample's s value. */
    val length: Double get() = sSamples.last()

    private val lastT get() = tSamples.last()
    /** @return the t on the original parametric function associated with [s] units along the curve. */
    fun tOfS(s: Double): Double = when {
        s <= 0.0 -> 0.0
        s >= length -> 1.0
        else -> run {
            var i = sSamples.binarySearch(s)
            if (i >= 0) return tSamples[i]
            i = -i - 2
            if (i >= numSamples - 1) return lastT
            val sBefore = sSamples[i]
            val tBefore = tSamples[i]

            val sAfter = sSamples[i + 1]
            val tAfter = tSamples[i + 1]
            val progress = (s - sBefore) / (sAfter - sBefore)
            return tBefore + progress * (tAfter - tBefore)
        }
    }

    /**
     * Gets a sequence of corresponding t parameter for all the s values given in [allS]
     * @see tOfS
     */
    fun getAllTOfS(allS: List<Double>): List<Double> {
        val firstS = allS.firstOrNull() ?: return emptyList()
        val firstSample = sSamples.binarySearch(firstS)
        var i = firstSample.replaceIf({ it < 0 }) { -it - 2 }
        return allS.map { s ->
            var sAfter: Double
            while (true) {
                if (i >= numSamples - 1) return@map lastT
                sAfter = sSamples[i + 1]
                if (s < sAfter) break
                i++
            }
            if (i == -1) return@map 0.0
            val sBefore = sSamples[i]
            val tBefore = tSamples[i]
            val tAfter = tSamples[i + 1]
            val progress = (s - sBefore) / (sAfter - sBefore)
            return@map tBefore + progress * (tAfter - tBefore)
        }
    }

    companion object {
        /**
         * Creates a reparam mapping from a list of (s,t) samples.
         */
        @JvmStatic
        fun fromSTpairs(samples: List<Pair<Double, Double>>): ReparamMapping {
            val size = samples.size
            val sSamples = DoubleArray(size) { samples[it].first }
            val tSamples = DoubleArray(size) { samples[it].second }
            return ReparamMapping(sSamples, tSamples)
        }

        /**
         * Creates a reparam mapping from a list of sSamples and corresponding tSamples.
         */
        @JvmStatic
        fun fromSTSamples(sSamples: List<Double>, tSamples: List<Double>): ReparamMapping {
            require(sSamples.size == tSamples.size)
            return ReparamMapping(sSamples.toDoubleArray(), tSamples.toDoubleArray())
        }

        /**
         * Creates a reparam mapping from a list of sSamples and corresponding tSamples.
         */
        @JvmStatic
        fun fromSTSamples(sSamples: DoubleArray, tSamples: DoubleArray): ReparamMapping {
            require(sSamples.size == tSamples.size)
            return ReparamMapping(sSamples.copyOf(), tSamples.copyOf())
        }
    }
}
