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
 * A [TimeProfiled] in which the velocity is always non-negative (position is always increasing).
 *
 * This way, it has a definite [distance] travelled, and the motion states can also be polled along [distance] as well
 * as time.
 *
 * @see TimeDistanceProfiled
 * @see MotionProfile
 */
interface ForwardMotionProfile : MotionProfile, TimeDistanceProfiled<RealMotionState>
