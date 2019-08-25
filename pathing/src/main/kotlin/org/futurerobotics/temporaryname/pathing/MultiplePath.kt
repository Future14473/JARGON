@file:Suppress("KDocMissingDocumentation")

package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.epsEq
import org.futurerobotics.temporaryname.util.Stepper
import org.futurerobotics.temporaryname.util.get
import org.futurerobotics.temporaryname.util.replaceIf

/**
 * Generified version of multiple Curve/Path a path made up of other segments.
 * Use [MultipleCurve] or [MultiplePath] instead.
 */
sealed class MultipleGeneric<Path : GenericPath<Point>, Point : CurvePoint>(
    paths: Iterable<Path>,
    checkContinuity: Boolean = true
) : GenericPath<Point> {

    private val paths: List<Path>

    init {
        this.paths = ArrayList<Path>(if (paths is Collection) paths.size else 10).also { list ->
            paths.forEach {
                if (it is MultipleGeneric<*, *>) list += it.paths.filterIsPath()
                else list += it
            }
        }
    }

    private val startLengths: DoubleArray
    final override val length: Double

    init {
        require(this.paths.isNotEmpty()) { "MultiplePath needs at least one path segment" }
        if (checkContinuity) paths.checkContinuity()
        startLengths = DoubleArray(this.paths.size)
        var curLen = 0.0
        this.paths.forEachIndexed { i, path ->
            startLengths[i] = curLen
            curLen += path.length
        }
        length = curLen
    }

    private inline val maxInd get() = paths.size - 1

    /** Gets a [Point] for a point [s] units along this path. */
    override fun atLength(s: Double): Point {
        val i = startLengths.binarySearch(s)
            .replaceIf({ it < 0 }) { -it - 2 }
            .coerceIn(0, maxInd)
        return paths[i].atLength(s - startLengths[i])
    }

    override fun stepper(): Stepper<Double, Point> = object :
        Stepper<Double, Point> {
        private var i = -1
        private lateinit var curStepper: Stepper<Double, Point>

        override fun stepTo(step: Double): Point = step.let { s ->
            val pastI = i
            if (i==-1) {
                i = startLengths.binarySearch(s)
                    .replaceIf({ it < 0 }) { -it - 2 }
                    .coerceIn(0, maxInd)
            } else {
                while (i < maxInd && s >= startLengths[i + 1]) i++

                while (i > 0 && s < startLengths[i]) i--
                //              Delegate extreme points behavior to the end paths
            }
            if (i != pastI) {
                curStepper = paths[i].stepper()
            }

            return curStepper[s - startLengths[i]] //ok if single threaded which is what we assume
        }
    }

    private fun Iterable<Path>.checkContinuity() {
        var prevPath: Path? = null
        for (curPath in this) {
            if (prevPath != null) {
                val prev = prevPath.atLength(prevPath.length)
                val cur = curPath.atLength(0.0)
                @Suppress("LeakingThis")
                checkPointContinuity(prev, cur)
            }
            prevPath = curPath
        }
    }

    protected abstract fun List<GenericPath<*>>.filterIsPath(): Iterable<Path>
    protected abstract fun checkPointContinuity(prev: Point, cur: Point)
}

/**
 * A [Curve] that consists of multiple other C-2 continuously connected [Curve]s.
 */
class MultipleCurve(paths: Iterable<Curve>, checkContinuity: Boolean = true) :
    MultipleGeneric<Curve, CurvePoint>(paths, checkContinuity),
    Curve {

    constructor(checkContinuity: Boolean = true, vararg curves: Curve) : this(curves.asList(), checkContinuity)

    override fun List<GenericPath<*>>.filterIsPath(): Iterable<Curve> = filterIsInstance<Curve>()
    override fun checkPointContinuity(prev: CurvePoint, cur: CurvePoint) {
        checkCont("Position", prev.position, cur.position)
        checkCont("PositionDeriv", prev.positionDeriv, cur.positionDeriv)
        checkCont(
            "PositionSecondDeriv",
            prev.positionSecondDeriv,
            cur.positionSecondDeriv
        )
        //tanAngle, tanAngleDeriv covered by above, tanAngleSecond deriv can be discontinuous.
    }
}

/**
 * A [Path] that consists of multiple other C-2 continuously connected [Path]s
 */
class MultiplePath(paths: Iterable<Path>, checkContinuity: Boolean = true) :
    MultipleGeneric<Path, PathPoint>(paths, checkContinuity),
    Path {
    override val isPointTurn: Boolean = paths.all { it.isPointTurn }

    constructor(checkContinuity: Boolean = true, vararg paths: Path) : this(paths.asList(), checkContinuity)

    override fun List<GenericPath<*>>.filterIsPath(): Iterable<Path> = filterIsInstance<Path>()

    override fun checkPointContinuity(prev: PathPoint, cur: PathPoint) {
        checkCont("Position", prev.position, cur.position)
        checkCont("PositionDeriv", prev.positionDeriv, cur.positionDeriv)
        checkCont(
            "PositionSecondDeriv",
            prev.positionSecondDeriv,
            cur.positionSecondDeriv
        )
        checkCont("Heading", prev.heading, cur.heading)
        checkCont(
            "HeadingDeriv/Curvature",
            prev.headingDeriv,
            cur.headingDeriv
        )
    }
}

private fun checkCont(name: String, prev: Vector2d, cur: Vector2d) {
    require(prev epsEq cur) { "$name discontinuity: $prev, $cur" }
}

private fun checkCont(name: String, prev: Double, cur: Double) {
    require(prev epsEq cur) { "$name discontinuity: $prev, $cur" }
}
