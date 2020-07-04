package org.futurerobotics.jargon.profile

//TODO: revise this to be better
/**
 * Mechanism to indicate a specific time along [TimeProfiled],
 * that can possibly differ when different profiles are given.
 *
 * This can be used by [TimeProfiledWithCallbacks].
 *
 * @see AtBegin
 * @see AtEnd
 * @see AtTime
 * @see AtDistance
 */
interface ProfileTimeIndicator<in P : TimeProfiled<*>> {

    /**
     * Gets an indicated time along the given [profiled] object (in which a callback should be triggered).
     */
    fun getTime(profiled: P): Double
}

/**
 * A [ProfileTimeIndicator] that indicates the beginning of the profile (time 0.0).
 */
class AtBegin : ProfileTimeIndicator<TimeProfiled<*>> {

    override fun getTime(profiled: TimeProfiled<*>): Double = 0.0
}

/**
 * A [ProfileTimeIndicator] that indicates the end of the profile (time = duration).
 */
class AtEnd : ProfileTimeIndicator<TimeProfiled<*>> {

    override fun getTime(profiled: TimeProfiled<*>): Double = profiled.duration
}

/**
 * A [ProfileTimeIndicator] that indicates a given [time].
 */
class AtTime(private val time: Double) :
    ProfileTimeIndicator<TimeProfiled<*>> {

    override fun getTime(profiled: TimeProfiled<*>): Double = time
}

/**
 * A [ProfileTimeIndicator] that returns a time at a given [distance].
 */
class AtDistance(private val distance: Double) :
    ProfileTimeIndicator<TimeDistanceProfiled<*>> {

    override fun getTime(profiled: TimeDistanceProfiled<*>): Double = profiled.timeAtDistance(distance)
}
