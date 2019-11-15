package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.asUnmodifiableSet
import org.futurerobotics.jargon.util.replaceIf
import org.futurerobotics.jargon.util.uncheckedCast

private sealed class MultipleGeneric<Path : GenericPath<Point>, Point : CurvePoint>
constructor(paths: List<Path>) : GenericPath<Point> {

    private val paths: List<Path>
    private val startLengths: DoubleArray
    final override val length: Double

    final override val stopPoints: Set<Double>

    init {
        require(paths.isNotEmpty()) { "MultiplePath needs at least one path segment" }
        val realPaths = ArrayList<Path>(paths.size)
        paths.forEach {
            if (it is MultipleGeneric<*, *>) realPaths += it.paths.uncheckedCast<List<Path>>()
            else realPaths += it
        }

        val stops = hashSetOf<Double>()
        startLengths = DoubleArray(realPaths.size)
        var curLen = 0.0
        var prevPath: Path? = null

        realPaths.forEachIndexed { i, curPath ->
            startLengths[i] = curLen
            prevPath?.let { prevPath ->
                if (needStop(prevPath, curPath)) stops += curLen
            }
            stops += curPath.stopPoints.map { it + curLen }
            curLen += curPath.length
            prevPath = curPath
        }
        length = curLen
        this.paths = realPaths
        this.stopPoints = stops.asUnmodifiableSet()
    }

    override val criticalPoints: Set<Double> = hashSetOf<Double>().let {
        it += stopPoints
        for (i in 1 until startLengths.size - 1) {
            it += startLengths[i]
        }
        it.asUnmodifiableSet()
    }
    private inline val maxInd get() = paths.size - 1

    /** Gets a [Point] for a point [s] units along this path. */
    override fun pointAt(s: Double): Point {
        val i = startLengths.binarySearch(s)
            .replaceIf({ it < 0 }) { -it - 2 }
            .coerceIn(0, maxInd)
        return paths[i].pointAt(s - startLengths[i])
    }

    override fun stepper(): Stepper<Double, Point> = object : Stepper<Double, Point> {
        private var i = -1
        private lateinit var curStepper: Stepper<Double, Point>
        override fun stepTo(step: Double): Point = step.let { s ->
            val pastI = i
            if (i == -1) {
                i = startLengths.binarySearch(s)
                    .replaceIf({ it < 0 }) { -it - 2 }
                    .coerceIn(0, maxInd)
            } else {
                while (i < maxInd && s >= startLengths[i + 1]) i++

                while (i > 0 && s < startLengths[i]) i--
            }
            if (i != pastI) {
                curStepper = paths[i].stepper()
            }

            return curStepper.stepTo(s - startLengths[i])
        }
    }

    private fun needStop(prevPath: Path, curPath: Path): Boolean {
        val prev = prevPath.pointAt(prevPath.length)
        val cur = curPath.pointAt(0.0)
        return checkPointContinuity(prev, cur)
    }

    protected open fun checkPointContinuity(prev: Point, cur: Point): Boolean {
        var stop = false
        checkCont("Position", prev.position, cur.position, true)
        stop = stop || checkCont("PositionDeriv", prev.positionDeriv, cur.positionDeriv, false)
        stop = stop || checkCont(
            "PositionSecondDeriv",
            prev.positionSecondDeriv,
            cur.positionSecondDeriv,
            false
        )
        return stop
    }
}

private class MultipleCurve(paths: List<Curve>) : MultipleGeneric<Curve, CurvePoint>(paths), Curve {

    companion object {
        private const val serialVersionUID: Long = -6008569752536122191
    }
}

private class MultiplePath(paths: List<Path>) : MultipleGeneric<Path, PathPoint>(paths), Path {

    override fun checkPointContinuity(prev: PathPoint, cur: PathPoint): Boolean {
        var stop = super.checkPointContinuity(prev, cur)
        checkCont("Heading", prev.heading, cur.heading, true)
        stop = stop || checkCont(
            "HeadingDeriv/Curvature",
            prev.headingDeriv,
            cur.headingDeriv,
            false
        )
        stop = stop || checkCont(
            "HeadingSecondDeriv",
            prev.headingSecondDeriv,
            cur.headingSecondDeriv,
            false
        )
        return stop
    }

    companion object {
        private const val serialVersionUID: Long = 1903180955913210312
    }
}

private fun checkCont(name: String, prev: Vector2d, cur: Vector2d, throwIfDisc: Boolean): Boolean {
    if (!(prev epsEq cur)) {
        require(!throwIfDisc) { "$name discontinuity: $prev, $cur" }
        return true
    }
    return false
}

private fun checkCont(name: String, prev: Double, cur: Double, throwIfDisc: Boolean): Boolean {
    if (!(prev epsEq cur)) {
        require(!throwIfDisc) { "$name discontinuity: $prev, $cur" }
        return true
    }
    return false
}

/**
 * Creates a new [Curve] which contains the given [curves] chained end on end.
 *
 * This must be at least continuous on position, and if not continuous on velocity, a point stop will be added.
 */
fun multipleCurve(curves: List<Curve>): Curve = when (curves.size) {
    0 -> throw IllegalArgumentException("Must contain at least 1 curve")
    1 -> curves.first()
    else -> MultipleCurve(curves)
}

/**
 * Creates a new [Curve] which contains the given [curves] chained end on end.
 *
 * This must be at least continuous on position, and if not continuous on velocity, a point stop will be added.
 */
fun multipleCurve(vararg curves: Curve): Curve = MultipleCurve(curves.asList())

/**
 * Creates a new [Path] which contains the given [paths] chained end on end.
 *
 * This must be at least continuous on position and heading, and if not continuous on velocity, a point stop will be
 * added.
 */
fun multiplePath(paths: List<Path>): Path = when (paths.size) {
    0 -> throw IllegalArgumentException("Must contain at least 1 path")
    1 -> paths.first()
    else -> MultiplePath(paths)
}

/**
 * Creates a new [Path] which contains the given [paths] chained end on end.
 *
 * This must be at least continuous on position and heading, and if not continuous on velocity, a point stop will be
 * added.
 */
fun multiplePath(vararg paths: Path): Path = MultiplePath(paths.asList())
