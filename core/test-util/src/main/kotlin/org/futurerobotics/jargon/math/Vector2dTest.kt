package org.futurerobotics.jargon.math

import kotlin.random.Random

/**
 * Creates a random vector
 * @param random the [Random] to use
 * @param range the range of values for x and y
 */
fun Random.nextVector2d(range: Double = 1.0): Vector2d =
    Vector2d(this.nextDouble(-range, range), this.nextDouble(-range, range))
