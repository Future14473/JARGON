package org.futurerobotics.temporaryname.control

/**
 * Simple random-holder class for Gaussian noise, with the given  standard deviation ([sd]) and mean 0
 */
class Noise(var sd: Double, seed: Long) {
    private val random = java.util.Random(seed)

    fun sample(): Double {
        return random.nextGaussian() * sd
    }
}