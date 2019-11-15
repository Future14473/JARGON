@file:JvmName("ReversePaths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.asUnmodifiableSet

private sealed class ReverseGeneric<Path : GenericPath<Point>, Point : CurvePoint>
constructor(internal val path: Path) : GenericPath<Point> {

    final override val length: Double get() = path.length
    override val stopPoints: Set<Double> get() = path.stopPoints.mapTo(HashSet()) { length - it }.asUnmodifiableSet()

    final override fun pointAt(s: Double): Point = mapPoint(path.pointAt(length - s))

    final override fun stepper(): Stepper<Double, Point> {
        val baseStepper = path.stepper()
        return Stepper { s ->
            mapPoint(baseStepper.stepTo(length - s))
        }
    }

    abstract fun mapPoint(point: Point): Point
}

private class ReverseCurve(curve: Curve) : ReverseGeneric<Curve, CurvePoint>(curve), Curve {

    override fun mapPoint(point: CurvePoint): CurvePoint = Point(point)

    private class Point(private val point: CurvePoint) : CurvePoint by point {
        override val positionDeriv: Vector2d
            get() = -point.positionDeriv
        override val tanAngleDeriv: Double
            get() = -point.tanAngleDeriv
    }

    companion object {
        private const val serialVersionUID = -8380484370027657370
    }
}

private class ReversePath(path: Path) : ReverseGeneric<Path, PathPoint>(path), Path {

    override fun mapPoint(point: PathPoint): PathPoint = Point(point)

    private class Point(private val point: PathPoint) : PathPoint by point {
        override val positionDeriv: Vector2d
            get() = -point.positionDeriv
        override val tanAngleDeriv: Double
            get() = -point.tanAngleDeriv
        override val headingDeriv: Double
            get() = -point.headingDeriv
    }

    companion object {
        private const val serialVersionUID = -9173494841041408460
    }
}

/**
 * Returns this curve, but traversed in the reverse direction.
 * First derivatives will be negated.
 */
fun Curve.reversed(): Curve =
    if (this is ReverseCurve) this.path
    else ReverseCurve(this)

/**
 * Returns this path, but traversed in the reverse direction.
 * First derivatives will be negated.
 */
fun Path.reversed(): Path =
    if (this is ReversePath) this.path
    else ReversePath(this)
