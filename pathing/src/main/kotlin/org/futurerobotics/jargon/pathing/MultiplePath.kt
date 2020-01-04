@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.asUnmodifiableSet
import org.futurerobotics.jargon.util.replaceIf
import org.futurerobotics.jargon.util.uncheckedCast
import java.util.*
import kotlin.collections.ArrayList
import kotlin.collections.HashSet

private sealed class MultipleGeneric<Path : GenericPath<Point>, Point : CurvePoint>
constructor(paths: List<Path>) : GenericPath<Point> {

    private val paths: List<Path>
    private val startLengths: DoubleArray

    final override val length: Double
    final override val stopPoints: Set<Double>
    final override val requiredPoints: Set<Double>

    init {
        require(paths.isNotEmpty()) { "MultiplePath needs at least one path segment" }
        val flatPaths = paths.flatMap {
            if (it is MultipleGeneric<*, *>)
                it.paths.uncheckedCast<List<Path>>()
            else Collections.singleton(it)
        }
        val realPaths = ArrayList<Path>()

        val capacity = flatPaths.size * 2
        var prevPath: Path? = null
        val startLengths = ArrayList<Double>(capacity)
        val stopPoints = HashSet<Double>(capacity)
        val requiredPoints = HashSet<Double>(capacity)
        var curLen = 0.0

        fun addPath(path: Path) {
            realPaths += path
            startLengths += curLen
            stopPoints += path.stopPoints.asSequence().map { it + curLen }
            requiredPoints += path.requiredPoints.asSequence().map { it + curLen }
            curLen += path.length
            requiredPoints += curLen
            prevPath = path
        }

        flatPaths.forEach { curPath ->
            prevPath?.let { prevPath ->
                when (val action = checkCont(prevPath, curPath)) {
                    is PathAction.AddPointTurn -> {
                        val turn = action.getTurn().uncheckedCast<Path>()
                        addPath(turn)
                    }
                    PathAction.Stop -> stopPoints += curLen
                }
            }
            addPath(curPath)
        }

        this.paths = realPaths
        this.startLengths = startLengths.toDoubleArray()
        this.length = curLen
        this.stopPoints = stopPoints.asUnmodifiableSet()
        this.requiredPoints = requiredPoints.asUnmodifiableSet()
    }

    /** Gets a [Point] for a point [s] units along this path. */
    override fun pointAt(s: Double): Point {
        val i = startLengths.binarySearch(s)
            .replaceIf({ it < 0 }) { -it - 2 }
            .coerceIn(0, paths.lastIndex)
        return paths[i].pointAt(s - startLengths[i])
    }

    override fun stepper(): Stepper<Point> = object : Stepper<Point> {
        private var i = -1
        private lateinit var curStepper: Stepper<Point>
        override fun stepTo(step: Double): Point = step.let { s ->
            val pastI = i
            if (i == -1) {
                i = startLengths.binarySearch(s)
                    .replaceIf({ it < 0 }) { -it - 2 }
                    .coerceIn(0, paths.lastIndex)
            } else {
                while (i < paths.lastIndex && s >= startLengths[i + 1]) i++

                while (i > 0 && s < startLengths[i]) i--
            }
            if (i != pastI) {
                curStepper = paths[i].stepper()
            }

            return curStepper.stepTo(s - startLengths[i])
        }
    }

    private fun checkCont(prevPath: Path, curPath: Path): PathAction {
        val prev = prevPath.pointAt(prevPath.length)
        val cur = curPath.pointAt(0.0)
        return checkPointContinuity(prev, cur)
    }

    protected open fun checkPointContinuity(prev: Point, cur: Point): PathAction {
        require(prev.position epsEq cur.position) { "Position discontinuity: ${prev.position}, ${cur.position}" }
        return if (!(prev.positionDeriv epsEq cur.positionDeriv)) PathAction.Stop
        else PathAction.None
    }

    protected sealed class PathAction {

        class AddPointTurn(val location: Vector2d, val from: Double, val to: Double) : PathAction() {
            fun getTurn() = PointTurn(location, from, angleNorm(to - from))
        }

        object None : PathAction()
        object Stop : PathAction()
    }
}

private class MultipleCurve(paths: List<Curve>) : MultipleGeneric<Curve, CurvePoint>(paths), Curve

private class MultiplePath(paths: List<Path>) : MultipleGeneric<Path, PathPoint>(paths), Path {

    override fun checkPointContinuity(prev: PathPoint, cur: PathPoint): PathAction {
        val superCheck = super.checkPointContinuity(prev, cur)
        if (!(angleNorm(prev.heading - cur.heading) epsEq 0.0)) {
            return PathAction.AddPointTurn(prev.position, prev.heading, cur.heading)
        }
        if (superCheck !== PathAction.Stop) return superCheck
        return if (!(prev.headingDeriv epsEq cur.headingDeriv)) PathAction.Stop else PathAction.None
    }
}

/**
 * Creates a new [Curve] which contains the given [curves] chained end on end.
 *
 * This must be at least continuous on position, and if not continuous on velocity, a point stop will be added.
 */
fun multipleCurve(curves: List<Curve>): Curve = when (curves.size) {
    0 -> throw IllegalArgumentException("Must be given at least 1 curve to multipleCurve")
    1 -> curves[0]
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
 * This must be at least continuous on position and heading. If direction or heading velocity is not continuous, a
 * point-stop will be added. If heading is not continuous, a [PointTurn] will be added.
 */
fun multiplePath(paths: List<Path>): Path = when (paths.size) {
    0 -> throw IllegalArgumentException("Must be given at least 1 path to multiplePath")
    1 -> paths[0]
    else -> MultiplePath(paths)
}

/**
 * Creates a new [Path] which contains the given [paths] chained end on end.
 *
 * This must be at least continuous on position and heading. If direction or heading velocity is not continuous, a
 * point-stop will be added. If heading is not continuous, a [PointTurn] will be added.
 */
fun multiplePath(vararg paths: Path): Path = MultiplePath(paths.asList())
