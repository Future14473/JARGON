@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.util.Stepper
import kotlin.math.PI

private class HeadingRotatedPath(val path: Path, val angle: Double) : Path by path {

    override fun pointAt(s: Double): PathPoint = Point(path.pointAt(s), angle)

    override fun stepper(): Stepper<PathPoint> {
        val baseStepper = path.stepper()
        return Stepper { s ->
            Point(baseStepper.stepTo(s), angle)
        }
    }

    private class Point(point: PathPoint, angle: Double) : PathPoint by point {
        override val heading: Double = angleNorm(point.heading + angle)
        override val pose: Pose2d = Pose2d(position, heading)
    }
}

/**
 * Returns a new path with the heading rotated by the given [angleRadians].
 */
tailrec fun Path.headingRotated(angleRadians: Double): Path {
    require(angleRadians.isFinite()) { "Angle ($angleRadians) must be finite" }
    val normAngle = angleNorm(angleRadians)
    return when {
        this is HeadingRotatedPath -> this.path.headingRotated(this.angle + normAngle)
        normAngle epsEq 0.0 -> this
        else -> HeadingRotatedPath(this, normAngle)
    }
}

/**
 * Returns this path with the heading rotated 180 degrees/PI radians (traversed backwards).
 */
fun Path.backwards(): Path = headingRotated(PI)

