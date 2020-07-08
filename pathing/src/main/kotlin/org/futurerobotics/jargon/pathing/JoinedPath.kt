@file:JvmMultifileClass
@file:JvmName("Paths")

package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.angleNorm
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.pathing.DiscontinuityBehavior.*
import org.futurerobotics.jargon.util.Stepper
import org.futurerobotics.jargon.util.asUnmodifiableSet

/**
 * Creates a new [Curve] which contains the given [curves] connected end on end.
 *
 * The curves must be continuous on position (no gaps between curves).
 *
 * @param sharpTurnBehavior behavior a sharp turn between two curves is found
 */
@JvmOverloads
fun joinCurves(
    curves: List<Curve>,
    sharpTurnBehavior: DiscontinuityBehavior = Stop
): Curve = when (curves.size) {
    0 -> throw IllegalArgumentException("At least 1 curve must be given to multipleCurve")
    1 -> curves[0]
    else -> JoinedCurve(curves, CurveDiscontinuityBehavior(sharpTurnBehavior))
}

/**
 * Creates a new [Curve] which contains the given [curves] connected end on end.
 *
 * The curves must be continuous on position (no gaps between curves).
 *
 * @param sharpTurnBehavior behavior a sharp turn between two curves is found
 */
@JvmOverloads
fun joinCurves(
    vararg curves: Curve,
    sharpTurnBehavior: DiscontinuityBehavior = Stop
): Curve = joinCurves(curves.asList(), sharpTurnBehavior)

/**
 * Creates a new [Path] which contains the given [paths] connected end on end.
 *
 * The paths must be continuous on position (no gaps between paths).
 *
 * @param sharpTurnBehavior behavior a sharp turn between two paths is found
 * @param headingDiscontinuityBehavior the behavior when a discontinuity in heading is found
 * @param headingDerivDiscontinuityBehavior the behavior when a discontinuity in heading derivative is found
 */
@JvmOverloads
fun joinPaths(
    paths: List<Path>,
    sharpTurnBehavior: DiscontinuityBehavior = Stop,
    headingDiscontinuityBehavior: HeadingDiscontinuityBehavior = HeadingDiscontinuityBehavior.Turn,
    headingDerivDiscontinuityBehavior: DiscontinuityBehavior = Stop
): Path = when (paths.size) {
    0 -> throw IllegalArgumentException("At least 1 path must be given to multiplePath")
    1 -> paths[0]
    else -> JoinedPath(
        paths,
        PathDiscontinuityBehavior(
            sharpTurnBehavior,
            headingDiscontinuityBehavior,
            headingDerivDiscontinuityBehavior
        )
    )
}

/**
 * Creates a new [Path] which contains the given [paths] chained end on end.
 *
 * The paths must be continuous on position (no gaps between paths).
 *
 * @param sharpTurnBehavior behavior a sharp turn between two paths is found
 * @param headingDiscontinuityBehavior the behavior when a discontinuity in heading is found
 * @param headingDerivDiscontinuityBehavior the behavior when a discontinuity in heading derivative is found
 */
@JvmOverloads
fun joinPaths(
    vararg paths: Path,
    sharpTurnBehavior: DiscontinuityBehavior = Stop,
    headingDiscontinuityBehavior: HeadingDiscontinuityBehavior = HeadingDiscontinuityBehavior.Turn,
    headingDerivDiscontinuityBehavior: DiscontinuityBehavior = Stop
): Path = joinPaths(
    paths.asList(),
    sharpTurnBehavior,
    headingDiscontinuityBehavior,
    headingDerivDiscontinuityBehavior
)

/**
 * The behavior of [joinCurves] and [joinPaths] when it encounters a sharp turn between
 * curves/paths.
 */
enum class DiscontinuityBehavior {

    /** Throws an exception. */
    Throw,

    /** Adds a stop point to the curve/path. This is the default. */
    Stop,

    /** Ignores the discontinuity.*/
    Ignore
}

/**
 * The behavior of [joinCurves] and [joinPaths] when it encounters a discontinuity in heading.
 */
enum class HeadingDiscontinuityBehavior {

    /** Throws an exception. */
    Throw,

    /** Adds a [PointTurn] to the path. This is the default. */
    Turn,

    /** Ignores the discontinuity.*/
    Ignore
}

/**
 * Exception thrown when a discontinuity is encountered and the behavior is to throw an exception
 */
class PathDiscontinuityException(message: String? = null) : Exception(message)

private open class CurveDiscontinuityBehavior(
    val direction: DiscontinuityBehavior
)

private class PathDiscontinuityBehavior(
    direction: DiscontinuityBehavior,
    val heading: HeadingDiscontinuityBehavior,
    val headingDeriv: DiscontinuityBehavior
) : CurveDiscontinuityBehavior(direction)

private abstract class AnyJoinedCurve<Path : AnyPath<Point>, Point : CurvePoint>
constructor(paths: List<Path>, discBehavior: CurveDiscontinuityBehavior) : AnyPath<Point> {

    private val paths: List<Path>
    private val startLengths: DoubleArray

    final override val length: Double
    final override val stopPoints: Set<Double>

    init {
        val realPaths = ArrayList<Path>()
        val capacity = paths.size * 2
        var prevPath: Path? = null
        val startLengths = ArrayList<Double>(capacity)
        val stopPoints = HashSet<Double>(capacity)
        var curLen = 0.0

        fun addPath(path: Path) {
            realPaths += path
            startLengths += curLen
            stopPoints += path.stopPoints.asSequence().map { it + curLen }

            curLen += path.length

            prevPath = path
        }

        paths.forEach { curPath ->
            prevPath?.let { prevPath ->
                when (val action = checkContinuity(prevPath, curPath, discBehavior)) {
                    is PathCorrection.AddPointTurn -> {
                        @Suppress("UNCHECKED_CAST")
                        addPath(action.getTurn() as Path)
                    }
                    PathCorrection.Stop -> stopPoints += curLen
                }
            }
            addPath(curPath)
        }

        this.paths = realPaths
        this.startLengths = startLengths.toDoubleArray()
        this.length = curLen
        this.stopPoints = stopPoints.asUnmodifiableSet()
    }

    override fun pointAt(s: Double): Point {
        val i = startLengths.binarySearch(s)
            .let { if (it < 0) -it - 2 else it }
            .coerceIn(0, paths.lastIndex)
        return paths[i].pointAt(s - startLengths[i])
    }

    override fun stepper(): Stepper<Point> = object : Stepper<Point> {
        private var i = -1
        private lateinit var curStepper: Stepper<Point>
        override fun stepTo(step: Double): Point {
            val pastI = i
            if (i == -1) {
                i = startLengths.binarySearch(step)
                    .let { if (it < 0) -it - 2 else it }
                    .coerceIn(0, paths.lastIndex)
            } else {
                while (i < paths.lastIndex && step >= startLengths[i + 1]) i++

                while (i > 0 && step < startLengths[i]) i--
            }
            if (i != pastI) {
                curStepper = paths[i].stepper()
            }

            return curStepper.stepTo(step - startLengths[i])
        }
    }

    private fun checkContinuity(
        prevPath: Path,
        curPath: Path,
        discBehavior: CurveDiscontinuityBehavior
    ): PathCorrection {
        val prev = prevPath.pointAt(prevPath.length)
        val cur = curPath.pointAt(0.0)
        return checkPointContinuity(prev, cur, discBehavior)
    }

    protected open fun checkPointContinuity(
        prev: Point,
        cur: Point,
        discBehavior: CurveDiscontinuityBehavior
    ): PathCorrection {
        requireCont(prev.position epsEq cur.position) { "Position discontinuity: ${prev.position}, ${cur.position}" }

        return if (!(prev.positionDeriv epsEq cur.positionDeriv)) {
            when (discBehavior.direction) {
                Throw -> throw PathDiscontinuityException(
                    "Direction discontinuity (sharp turn): ${prev.positionDeriv}, ${cur.positionDeriv}"
                )
                Stop -> PathCorrection.Stop
                Ignore -> PathCorrection.None
            }
        } else PathCorrection.None
    }

    protected sealed class PathCorrection {

        class AddPointTurn(val location: Vector2d, val from: Double, val to: Double) : PathCorrection() {
            fun getTurn() = PointTurn(location, from, angleNorm(to - from))
        }

        object Stop : PathCorrection()
        object None : PathCorrection()
    }
}

private inline fun requireCont(value: Boolean, lazyMessage: () -> Any) {
    if (!value) {
        val message = lazyMessage()
        throw PathDiscontinuityException(message.toString())
    }
}

private class JoinedCurve(paths: List<Curve>, discBehavior: CurveDiscontinuityBehavior) :
    AnyJoinedCurve<Curve, CurvePoint>(paths, discBehavior), Curve

private class JoinedPath(paths: List<Path>, discBehavior: PathDiscontinuityBehavior) :
    AnyJoinedCurve<Path, PathPoint>(paths, discBehavior), Path {

    override fun checkPointContinuity(
        prev: PathPoint,
        cur: PathPoint,
        discBehavior: CurveDiscontinuityBehavior
    ): PathCorrection {
        //super call might throw
        val pathDiscBehavior = discBehavior as PathDiscontinuityBehavior
        val superCheck = super.checkPointContinuity(prev, cur, discBehavior)
        if (!(angleNorm(prev.heading - cur.heading) epsEq 0.0)) {
            when (pathDiscBehavior.heading) {
                HeadingDiscontinuityBehavior.Throw ->
                    throw PathDiscontinuityException(
                        "Heading discontinuity: ${prev.heading}, ${cur.heading}"
                    )
                HeadingDiscontinuityBehavior.Turn ->
                    return PathCorrection.AddPointTurn(
                        prev.position,
                        prev.heading,
                        cur.heading
                    )
                HeadingDiscontinuityBehavior.Ignore -> { //proceed
                }
            }
        }
        if (!(prev.headingDeriv epsEq cur.headingDeriv)) {
            when (pathDiscBehavior.headingDeriv) {
                Throw -> throw PathDiscontinuityException(
                    "Heading deriv discontinuity: ${prev.headingDeriv}, ${cur.headingDeriv}"
                )
                Stop -> return PathCorrection.Stop
                Ignore -> { //proceed
                }
            }
        }
        return superCheck
    }
}
