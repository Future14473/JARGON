package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.ComponentVectorFunction
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.nextVector2d
import kotlin.random.Random

/**
 * Creates a random set of [MotionState] of [Vector2d]
 */
fun randomVectorDerivatives(random: Random, range: Double): MotionState<Vector2d> =
    MotionState(
        random.nextVector2d(range), random.nextVector2d(range), random.nextVector2d(range)
    )

/**
 * Creates a random [QuinticSpline].
 */
fun randomQuinticSpline(random: Random, range: Double): ComponentVectorFunction = QuinticSpline.fromDerivatives(
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range),
    random.nextVector2d(range)
)
