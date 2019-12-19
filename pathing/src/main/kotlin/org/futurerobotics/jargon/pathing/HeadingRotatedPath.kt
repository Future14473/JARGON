@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.util.Stepper
import kotlin.math.PI

private class HeadingRotatedPath(internal val path: Path, internal val angle: Double) : Path by path {

    override fun pointAt(s: Double): PathPoint = Point(path.pointAt(s), angle)

    override fun stepper(): Stepper<PathPoint> {
        val baseStepper = path.stepper()
        return Stepper { s ->
            Point(baseStepper.stepTo(s), angle)
        }
    }

    private class Point(point: PathPoint, angle: Double) : PathPoint by point {
        override val heading: Double = angleNorm(point.heading + angle)
    }
}

/**
 * Returns this path with the heading rotated PI radians (backwards).
 */
fun Path.backwards(): Path = headingRotated(PI)

/**
 * Returns this path with the heading rotated by the given [angle].
 */
tailrec fun Path.headingRotated(angle: Double): Path {
    require(angle.isFinite()) { "Angle ($angle) must be finite" }
    val angle = angleNorm(angle)
    return when {
        this is HeadingRotatedPath -> headingRotated(this.angle + angle)
        angle epsEq 0.0 -> this
        else -> HeadingRotatedPath(this, angle)
    }
}

