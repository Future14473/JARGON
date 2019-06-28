package org.futurerobotics.temporaryname.math

/**
 * Assuming that [partition] will return true for values higher than `x` and false for values lower (`x` is a partition
 * point), this attempts to find a value within [tolerance] of `x` within the bounds [rangeMin], [rangeMax] that
 * evaluates to [searchingFor] when applied with [partition] (if false, a value at most [tolerance] below x, else value at
 * most [tolerance] above x).
 *
 * If, however, the partition point does not exist within the given range, one of the endpoints will be returned instead.
 *
 * This behaves with the same contract that [doubleBinarySearch] does, however: This is faster when given the heuristic
 * that the partition point is close to [rangeMin]. For best performance, [initialStep] should be slightly smaller than
 * the estimated difference from the partition point to [rangeMin].
 *
 * See [extendingDownDoubleSearch] for a version that works with the heuristic that the partition point is close to
 * [rangeMax] instead of [rangeMin]
 *
 * This works by first looking at the interval starting at [rangeMin], and upwards a step of [initialStep].
 * If both endpoints returns false (`x` is not in this interval), this then looks at another interval from the end of the
 * old interval and a step double the size. This is repeated until an interval switches [partition] is found (`x` is in
 * the interval), then performs normal binary search on the new interval.
 *
 * _This has not been tested with extreme values (where excessive rounding may occur). It is the user's responsibility
 * to make sure that the Double values provided are in close enough of magnitude._
 *
 * This takes `O(log((x-initialValue)/initialStep)+log(x-initialValue)/tolerance)))`
 * @see extendingDownDoubleSearch
 */
inline fun extendingDoubleSearch(
    rangeMin: Double,
    rangeMax: Double,
    initialStep: Double,
    tolerance: Double = initialStep,
    searchingFor: Boolean = false,
    partition: (Double) -> Boolean
): Double {
    require(rangeMin <= rangeMax) { "rangeMin ($rangeMin) must be <= rangeMax ($rangeMax)" }
    require(initialStep > 0) { "initialStep ($initialStep) must be > 0" }
    require(tolerance > 0) { "tolerance ($tolerance) must be > 0" }
    if (rangeMin == rangeMax) return rangeMin
    if (partition(rangeMin)) return rangeMin
    var lower = rangeMin
    var step = initialStep
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
 * This works the same as [extendingDoubleSearch] of which whose documentation you should probably read., but starts at the upper endpoint and works down.
 * Heuristic/runtime is flipped for the upper end instead.
 *
 * _This has not been tested with extreme values (where excessive rounding may occur). It is the user's responsibility
 * to make sure that the Double values provided are in close enough of magnitude._
 */
inline fun extendingDownDoubleSearch(
    rangeMin: Double,
    rangeMax: Double,
    initialStep: Double,
    tolerance: Double = initialStep,
    searchingFor: Boolean = false,
    partition: (Double) -> Boolean
): Double {
    require(rangeMin <= rangeMax) { "rangeMin ($rangeMin) must be <= rangeMax ($rangeMax)" }
    require(initialStep > 0) { "initialStep ($initialStep) must be > 0" }
    require(tolerance > 0) { "tolerance ($tolerance) must be > 0" }
    if (rangeMax == rangeMin) return rangeMax
    var upper = rangeMax
    if (!partition(rangeMax)) return rangeMax
    var step = initialStep
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
 * _This has not been tested with extreme values (where excessive rounding may occur). It is the user's responsibility
 * to make sure that the Double values provided are in close enough of magnitude._
 *
 * This is done using binary search.
 */
inline fun doubleBinarySearch(
    rangeMin: Double, rangeMax: Double, tolerance: Double, searchingFor: Boolean = false, partition: (Double) -> Boolean
): Double {
    require(rangeMax >= rangeMin) { "rangeMax ($rangeMax) must be >= rangeMin ($rangeMin)" }
    require(tolerance > 0) { "tolerance ($tolerance) must be >= 0" }
    if (partition(rangeMin)) return rangeMin
    if (!partition(rangeMax)) return rangeMax
    var lower = rangeMin
    var upper = rangeMax
    while (upper - lower > tolerance) {
        val mid = avg(lower, upper)
        if (partition(mid)) upper = mid
        else lower = mid
    }
    return if (!searchingFor) lower else upper
}

/**
 * Assuming that [partition] will return true for values higher greater than or equal to `x` and false for values lower,
 * this uses an "extending" search to find `x` if it exists in the range [rangeMin]..[rangeMax].
 * If 'x' is at or below [rangeMin], [rangeMin] will be returned.
 * If 'x' is above [rangeMax], [rangeMax]+1 will be returned.
 *
 * This works by first looking at the interval starting at [rangeMin], and upwards a step of 1.
 * If both endpoints returns false (`x` is not in this interval), this then looks at another interval from the end of the
 * old interval and a step double the size. This is repeated until an interval switches [partition] is found (`x` is in
 * the interval), then performs normal binary search on the new interval.
 *
 * _This has not been tested with extreme values (close to integer overflow)_.
 *
 * This takes `O(log(x-initialValue))`
 * @see extendingDownDoubleSearch
 */
inline fun extendingSearch(rangeMin: Int, rangeMax: Int, partition: (Int) -> Boolean): Int {
    require(rangeMax >= rangeMin) { "rangeMax ($rangeMax) must be >= rangeMin ($rangeMin)" }
    if (partition(rangeMin)) return rangeMin
    if (!partition(rangeMax)) return rangeMax + 1
    var lower = rangeMin
    var step = 1
    var upper: Int
    while (true) {
        upper = lower + step
        if (step < 0 || upper <= lower || upper >= rangeMax) { //include overflow.
            upper = rangeMax
            break
        }
        if (partition(upper)) break
        lower = upper
        step *= 2
    }
    while (upper != lower) {
        val mid = (lower + upper) / 2 //round down
        if (partition(mid)) upper = mid
        else lower = mid + 1
    }
    return lower
}