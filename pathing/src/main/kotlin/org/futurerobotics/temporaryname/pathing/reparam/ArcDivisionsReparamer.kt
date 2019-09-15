@file:Suppress("DEPRECATION")

package org.futurerobotics.temporaryname.pathing.reparam

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.avg
import org.futurerobotics.temporaryname.math.epsEq
import org.futurerobotics.temporaryname.math.function.VectorFunction
import org.futurerobotics.temporaryname.math.maxDiff
import org.futurerobotics.temporaryname.pathing.reparam.ArcDivisionsReparamer.Companion.reparam
import kotlin.math.abs
import kotlin.math.asin

/**
 * Builder class that reparameterizes an Curve by subdividing it into arcs. Very similar to ACME Robotic's version.
 * The function that actually does the reparameterization is [reparam]
 *
 * This has experimentally been determined to not be as great as [ArcDivisionsReparamer]
 */
@Deprecated("ArcDivisionsReparamer works a lot better")
class ArcDivisionsReparamer private constructor(
    private val curve: VectorFunction,
    private val maxDeltaK: Double,
    private val maxSegmentLength: Double,
    private val curvatureTolerance: Double
) {

    /** current length */
    private var curLength: Double = 0.0
    /** currently being built samples */
    private val sSamples = mutableListOf(0.0)
    private val tSamples = mutableListOf(0.0)
    private fun doReparam(): ReparamCurve {
        reparamOn(0.0, 1.0)
        return ReparamCurve(
            curve,
            SamplesReparamMapping.fromPointSamples(
                sSamples,
                tSamples
            )
        )
    }

    private fun reparamOn(
        t1: Double,
        t2: Double,
        c1: Double = curve.curvature(t1),
        c2: Double = curve.curvature(t2),
        p1: Vector2d = curve.vec(t1),
        p2: Vector2d = curve.vec(t2)
    ) {
        val tm = avg(t1, t2)
        val cm = curve.curvature(tm)
        val pm = curve.vec(tm)
        val deltaK = maxDiff(c1, cm, c2)
        if (deltaK <= maxDeltaK) {
            val avgK = avg(c1, cm, c2)
            val estLength = approxLengthAndCompare(p1, pm, p2, avgK)
            if (!estLength.isNaN() && estLength <= maxSegmentLength) {
                curLength += estLength
                sSamples.add(curLength)
                tSamples.add(t2)
                return
            }
        }
        reparamOn(t1, tm, c1, cm, p1, pm)
        reparamOn(tm, t2, cm, c2, pm, p2)
    }

    private fun approxLengthAndCompare(
        p1: Vector2d, p2: Vector2d, p3: Vector2d, expectedCurvature: Double
    ): Double {
        //thanks stack exchange
        //want to find center of circle going through 3 points, with matrices
        //shift points by p3 so that p3 is at (0,0), simplifies calculation
        val v1 = p1 - p3
        val v2 = p2 - p3
        //v3 = p3-p3 = 0
        val det = v1 cross v2 //so it happens
        val chordLen = v1.length
        return if (det epsEq 0.0) {
            chordLen //straight enough line
        } else {
            val e1 = v1.lengthSquared
            val e2 = v2.lengthSquared
            //center of circle from 3 points, times 2.
            val center2 = Vector2d(e1 * v2.y - e2 * v1.y, -(e1 * v2.x - e2 * v1.x)) / det
            //radius * 2
            val radius2 = center2.length //since one of the points is (0,0)
            if (abs(expectedCurvature - 2 / radius2) < curvatureTolerance
            ) radius2 * asin(chordLen / radius2) //arc length
            else Double.NaN
            //== 2 * radius * asin(chordLen / 2 / radius)
        }
    }

    companion object {
        /** Default maxDeltaK used for [ArcDivisionsReparamer] for overloads/default parameters. */
        const val defaultMaxDeltaK: Double = 0.5
        /** Default maxSegmentLength used for reparameterization for overloads/default parameters. */
        const val defaultMaxSegmentLength: Double = 0.05
        /** Default tolerance used for reparameterization for overloads/default parameters. */
        const val defaultCurvatureTolerance: Double = 0.2

        /**
         * Reparameterizes a [VectorFunction] [func] by subdividing the curves into smaller and smaller arcs until all of the following:
         * - the difference in curvature between the endpoints is less than [maxDeltaK]
         * - the segment length is less than [maxSegmentLength]
         * - the expected curvature from the function and approximate curvature calculated from points differs by no
         *     more than [curvatureTolerance]
         */
        @JvmStatic
        @JvmOverloads
        fun reparam(
            func: VectorFunction,
            maxDeltaK: Double = defaultMaxDeltaK,
            maxSegmentLength: Double = defaultMaxSegmentLength,
            curvatureTolerance: Double = defaultCurvatureTolerance
        ): ReparamCurve = ArcDivisionsReparamer(
            func, maxDeltaK, maxSegmentLength, curvatureTolerance
        ).doReparam()
    }
}

/** Convenience extension function for [ArcDivisionsReparamer.reparam] */
@Deprecated(
    "ArcDivisionsReparamer works a lot better.",
    ReplaceWith("reparamByIntegration()", "org.futurerobotics.temporaryname.pathing.reparam.reparamByIntegration")
)
fun VectorFunction.reparamByArcSubdivisions(
    maxDeltaK: Double = ArcDivisionsReparamer.defaultMaxDeltaK,
    maxSegmentLength: Double = ArcDivisionsReparamer.defaultMaxSegmentLength,
    curvatureTolerance: Double = ArcDivisionsReparamer.defaultCurvatureTolerance
): ReparamCurve = reparam(this, maxDeltaK, maxSegmentLength, curvatureTolerance)
