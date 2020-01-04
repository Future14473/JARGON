@file:Suppress("DuplicatedCode")

package org.futurerobotics.jargon.util

import org.futurerobotics.jargon.math.avg

/**
 * Assuming that [partition] will return true for values higher than `x` and false for values lower (`x` is a partition
 * point), this attempts to find a value within [tolerance] of `x` within the bounds [rangeMin], [rangeMax] that
 * evaluates to [searchingFor] when applied with [partition] (if false, a value at most [tolerance] below x, else value at
 * most [tolerance] above x).
 *
 * If, however, the partition point does not exist within the given range, one of the endpoints will be returned instead.
 *
 * This behaves with the same contract that [doubleBinarySearch] does, however, this may be faster when given the
 * heuristic that the partition point is close to [rangeMin].
 *
 * See [extendingDownDoubleSearch] for a version that works with the heuristic that the partition point is close to
 * [rangeMax] instead of [rangeMin]
 *
 * This works by first looking at the interval starting at [rangeMin], and upwards a step of [tolerance].
 * If both endpoints returns false (`x` is not in this interval), this then looks at another interval from the end of the
 * old interval, and a step double the size. This is repeated until an interval switches [partition] is found (`x` is in
 * the interval), then performs normal binary search on the new interval.
 *
 * _This has not been tested with extreme values (where excessive rounding may occur)._
 *
 * This takes `O(log((x-initialValue)/tolerance))`
 * @see extendingDownDoubleSearch
 */
inline fun extendingDoubleSearch(
    rangeMin: Double,
    rangeMax: Double,
    tolerance: Double,
    searchingFor: Boolean = false,
    partition: (Double) -> Boolean
): Double {
    require(rangeMin <= rangeMax) { "rangeMin ($rangeMin) must be <= rangeMax ($rangeMax)" }
    require(tolerance > 0) { "tolerance ($tolerance) must be > 0" }
    if (rangeMin == rangeMax) return rangeMin
    var lower = rangeMin
    var step = tolerance
    var upper: Double
    while (true) {
        upper = lower + step
        if (upper >= rangeMax) {
            upper = rangeMax
            break
        }
        if (partition(upper)) break
        lower = upper
        step *= 2
    }
    while (upper - lower > tolerance) {
        val mid = avg(lower, upper)
        if (partition(mid)) upper = mid
        else lower = mid
    }
    return if (!searchingFor) lower else upper
}

/**
 * This works the same as [extendingDoubleSearch] of which the documentation you should probably read,
 * but starts at the upper endpoint and works down. Heuristic/runtime is flipped for the upper end instead.
 *
 * _This has not been tested with extreme values (where excessive rounding may occur)._
 */
inline fun extendingDownDoubleSearch(
    rangeMin: Double,
    rangeMax: Double,
    tolerance: Double,
    searchingFor: Boolean = false,
    partition: (Double) -> Boolean
): Double {
    require(rangeMin <= rangeMax) { "rangeMin ($rangeMin) must be <= rangeMax ($rangeMax)" }
    require(tolerance > 0) { "tolerance ($tolerance) must be > 0" }
    if (rangeMin == rangeMax) return rangeMax
    var upper = rangeMax
    var step = tolerance
    var lower: Double
    while (true) {
        lower = upper - step
        if (lower <= rangeMin) {
            lower = rangeMin
            break
        }
        if (!partition(lower)) break
        upper = lower
        step *= 2
    }
    while (upper - lower > tolerance) {
        val mid = avg(lower, upper)
        if (partition(mid)) upper = mid
        else lower = mid
    }
    return if (!searchingFor) lower else upper
}

/**
 * Assuming that [partition] will return true for values higher than `x` and
 * false for values lower, this attempts to find a value within [tolerance] of `x`, that when evaluated with [partition]
 * to the value [searchingFor] (if false, a value at most tolerance below x, else value at most tolerance above x)
 *
 * If, however, the partition point does not exist within the given range, one of the endpoints will be returned instead.
 *
 * _This has not been tested with extreme values (where excessive rounding may occur)._
 *
 * This is done using binary search.
 */
inline fun doubleBinarySearch(
    rangeMin: Double, rangeMax: Double, tolerance: Double, searchingFor: Boolean = false, partition: (Double) -> Boolean
): Double {
    require(rangeMax >= rangeMin) { "rangeMax ($rangeMax) must be >= rangeMin ($rangeMin)" }
    require(tolerance > 0) { "tolerance ($tolerance) must be >= 0" }
    var lower = rangeMin
    var upper = rangeMax
    while (upper - lower > tolerance) {
        val mid = avg(lower, upper)
        if (partition(mid)) upper = mid
        else lower = mid
    }
    return if (!searchingFor) lower else upper
}
