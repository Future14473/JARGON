package org.futurerobotics.jargon.math

/** Extracts a vector [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.vec(): MotionState<Vector2d> =
    MotionState(value.vec, deriv.vec, secondDeriv.vec)

/** Extracts a heading [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.heading(): LinearMotionState =
    LinearMotionState(value.heading, deriv.heading, secondDeriv.heading)

/** Extracts a x [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.x(): LinearMotionState =
    LinearMotionState(value.x, deriv.x, secondDeriv.x)

/** Extracts the y [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.y(): LinearMotionState =
    LinearMotionState(value.y, deriv.y, secondDeriv.y)
