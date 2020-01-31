package org.futurerobotics.jargon.math

/** Extracts a vector [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.vec(): MotionState<Vector2d> =
    MotionState(value.vector2d, deriv.vector2d, secondDeriv.vector2d)

/** Extracts a heading [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.heading(): RealMotionState =
    RealMotionState(value.heading, deriv.heading, secondDeriv.heading)

/** Extracts a x [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.x(): RealMotionState =
    RealMotionState(value.x, deriv.x, secondDeriv.x)

/** Extracts the y [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.y(): RealMotionState =
    RealMotionState(value.y, deriv.y, secondDeriv.y)
