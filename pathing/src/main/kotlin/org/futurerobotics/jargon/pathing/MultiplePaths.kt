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
import kotlin.math.roundToInt

private sealed class MultipleGeneric<Path : GenericPath<Point>, Point : CurvePoint>
constructor(paths: List<Path>) : GenericPath<Point> {

    private val paths: List<Path>
    private val startLengths: DoubleArray
    final override val length: Double
    final override val stopPoints: Set<Double>

    init {
        require(paths.isNotEmpty()) { "MultiplePath needs at least one path segment" }
        val flatPaths = ArrayList<Path>(paths.size)
        paths.forEach {
            if (it is MultipleGeneric<*, *>) flatPaths += it.paths.uncheckedCast<List<Path>>()
            else flatPaths += it
        }

        val stops = hashSetOf<Double>()
        val initialCapacity = (flatPaths.size * 1.05).roundToInt()
        val startLengths = ArrayList<Double>(initialCapacity)
        val realPaths = ArrayList<Path>(initialCapacity)

        var curLen = 0.0
        var prevPath: Path? = null
        flatPaths.forEach { curPath ->
            prevPath?.let { prevPath ->
                when (val action = checkCont(prevPath, curPath)) {
                    is PathAction.AddPointTurn -> {
                        val turn = action.getTurn().uncheckedCast<Path>()
                        realPaths += turn
                        startLengths += curLen
                        stops += turn.stopPoints.map { it + curLen }
                        curLen += turn.length
                    }
                    PathAction.Stop -> stops += curLen
                }
            }
            //dup code... any way around this?
            realPaths += curPath
            startLengths += curLen
            stops += curPath.stopPoints.map { it + curLen }
            curLen += curPath.length
            prevPath = curPath
        }
        length = curLen
        this.paths = realPaths
        this.stopPoints = stops.asUnmodifiableSet()
        this.startLengths = startLengths.toDoubleArray()
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

    override fun stepper(): Stepper<Point> = object : Stepper<Point> {
        private var i = -1
        private lateinit var curStepper: Stepper<Point>
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

private class MultipleCurve(paths: List<Curve>) : MultipleGeneric<Curve, CurvePoint>(paths), Curve {

    companion object {
        private const val serialVersionUID: Long = -6008569752536122191
    }
}

private class MultiplePath(paths: List<Path>) : MultipleGeneric<Path, PathPoint>(paths), Path {

    override fun checkPointContinuity(prev: PathPoint, cur: PathPoint): PathAction {
        val superCheck = super.checkPointContinuity(prev, cur)
        if (!(angleNorm(prev.heading - cur.heading) epsEq 0.0)) {
            return PathAction.AddPointTurn(prev.position, prev.heading, cur.heading)
        }
        if (superCheck !== PathAction.Stop) return superCheck
        return if (!(prev.headingDeriv epsEq cur.headingDeriv)) PathAction.Stop else PathAction.None
    }

    companion object {
        private const val serialVersionUID: Long = 1903180955913210312
    }
}

/**
 * Creates a new [Curve] which contains the given [curves] chained end on end.
 *
 * This must be at least continuous on position, and if not continuous on velocity, a point stop will be added.
 */
fun multipleCurve(curves: List<Curve>): Curve = when (curves.size) {
    0 -> throw IllegalArgumentException("Must contain at least 1 curve")
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
 * This must be at least continuous on position and heading, and if not continuous on velocity, a point stop will be
 * added.
 */
fun multiplePath(paths: List<Path>): Path = when (paths.size) {
    0 -> throw IllegalArgumentException("Must contain at least 1 path")
    1 -> paths[0]
    else -> MultiplePath(paths)
}

/**
 * Creates a new [Path] which contains the given [paths] chained end on end.
 *
 * This must be at least continuous on position and heading, and if not continuous on velocity, a point stop will be
 * added.
 */
fun multiplePath(vararg paths: Path): Path = MultiplePath(paths.asList())
