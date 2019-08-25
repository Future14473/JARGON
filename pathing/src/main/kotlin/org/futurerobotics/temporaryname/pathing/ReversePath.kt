package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.util.Stepper

private sealed class ReverseGeneric<Path : GenericPath<Point>, Point : CurvePoint>(internal val path: Path) :
    GenericPath<Point> {
    final override val length: Double get() = path.length
    final override fun atLength(s: Double): Point {
        return mapPoint(path.atLength(length - s))
    }

    final override fun stepper(): Stepper<Double, Point> {
        val baseStepper = path.stepper()
        return Stepper { s ->
            mapPoint(baseStepper.stepTo(length - s))
        }
    }

    //factory method! I'm proud of myself.
    abstract fun mapPoint(point: Point): Point
}

private open class ReversePoint<Point : CurvePoint>(protected val point: Point) : CurvePoint by point {
    override val positionDeriv: Vector2d
        get() = -point.positionDeriv
    override val tanAngleDeriv: Double
        get() = -point.tanAngleDeriv
}
private typealias ReverseCurvePoint = ReversePoint<CurvePoint>

private class ReversePathPoint(point: PathPoint) : ReversePoint<PathPoint>(point),
    PathPoint {
    override val heading: Double
        get() = point.heading
    override val headingDeriv: Double
        get() = -point.headingDeriv
    override val headingSecondDeriv: Double
        get() = point.headingSecondDeriv
}

private class ReverseCurve(curve: Curve) : ReverseGeneric<Curve, CurvePoint>(curve),
    Curve {
    override fun mapPoint(point: CurvePoint): CurvePoint =
        ReverseCurvePoint(point)
}

private class ReversePath(path: Path) : ReverseGeneric<Path, PathPoint>(path),
    Path {
    override fun mapPoint(point: PathPoint): PathPoint =
        ReversePathPoint(point)
    override val isPointTurn: Boolean
        get() = path.isPointTurn
}

/**
 * Returns this curve, but traversed in the reverse direction.
 * First derivatives will be negated.
 */
fun Curve.reversed(): Curve = if (this is ReverseGeneric<*, *>) this.path else ReverseCurve(
    this
)

/**
 * Returns this Path, but traversed in the reverse direction.
 * First derivatives will be negated.
 */
fun Path.reversed(): Curve = if (this is ReversePath) this.path else ReversePath(
    this
)
