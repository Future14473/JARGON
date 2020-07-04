package org.futurerobotics.jargon.profile

import org.futurerobotics.jargon.math.RealMotionState

/**
 * Represents a motion profile: a graph/profile of velocity (and position and acceleration) over time.
 *
 * This is also a [TimeProfiled] with [RealMotionState]s.
 *
 * @see TimeProfiled
 */
interface MotionProfile : TimeProfiled<RealMotionState>

/**
 * A [MotionProfile] in which the velocity is always non-negative (so the position is always increasing).
 *
 * As such, specific points in time can be uniquely determined from a given [distance] travelled, and hence
 * the motion states can also be polled along [distance] as well as time.
 *
 * @see TimeDistanceProfiled
 * @see MotionProfile
 */
interface ForwardMotionProfile : MotionProfile, TimeDistanceProfiled<RealMotionState>
