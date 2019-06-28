package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.Vector2d
import org.futurerobotics.temporaryname.math.epsEq

/**
 * Represents a path for a robot to follow, defined by a list of C2-continuously connected [PathSegment]s.
 * Will check for continuity
 */
class Path(segments: List<PathSegment>) {
    constructor(vararg segments: PathSegment) : this(segments.asList())

    private val segments: List<PathSegment> = segments.toList()
    private val startLengths: DoubleArray

    /** The total length of this path. */
    val length: Double

    init {
        require(this.segments.isNotEmpty()) { "Path needs at least one segment" }
        checkContinuity(this.segments)
        startLengths = DoubleArray(this.segments.size)
        var totalLen = 0.0
        this.segments.forEachIndexed { i, s ->
            startLengths[i] = totalLen
            totalLen += s.length
        }
        length = totalLen
    }


    /** Gets a [PathPointInfo] for a point [s] units along this path. */
    fun getPointInfo(s: Double): PathPointInfo = when {
        s <= 0.0 -> segments[0].getPointInfo(0.0)
        s >= length -> segments.last().let { it.getPointInfo(it.length) }
        else -> run {
            var i = startLengths.binarySearch(s)
            if (i < 0) i = -i - 2
            return segments[i].getPointInfo(s - startLengths[i])
        }
    }

    /**
     * Gets a list of [PathPointInfo] for a sorted list of points s units along the path given by [allS]
     *
     * If `allS` list is not sorted, the behavior is undefined.
     */
    fun getAllPointInfo(allS: List<Double>): List<PathPointInfo> {
        val firstS = allS.firstOrNull() ?: return emptyList()
        val size = allS.size
        val result = ArrayList<PathPointInfo>(size)
        var segInd = startLengths.binarySearch(firstS) //first seg
        if (segInd < 0) segInd = -segInd - 2
        var allSInd = 0
        // <= 0.0
        if (segInd == -1 && firstS <= 0) {
            val firstPointInfo = segments[0].getPointInfo(0.0)
            while (allS[allSInd] <= 0) {
                result += firstPointInfo
                allSInd++
                if (allSInd == size) break
            }
            segInd = 0
        }
        //0 < s < length
        for (segInd in segInd until segments.size) { //by segs
            if (allSInd == size) break
            val segmentStartInd = allSInd
            val endLen = if (segInd < segments.size - 1) startLengths[segInd + 1] else length
            //linear search endpoint for values that fall in this segment
            while (allSInd < size && allS[allSInd] < endLen) { //if equal, do next seg.
                allSInd++
            }
            if (segmentStartInd == allSInd) continue
            val segmentEndInd = allSInd
            val startLen = startLengths[segInd]
            val sublist = allS.subList(segmentStartInd, segmentEndInd).map { it - startLen }
            result += segments[segInd].getAllPointInfo(sublist)
        }
        // >= length
        if (allSInd < size) {
            val lastPointInfo = segments.last().let { it.getPointInfo(it.length) }
            repeat(size - allSInd) {
                result += lastPointInfo
            }
        }
        return result
    }
}

/**
 * Creates a Path consisting only of this segment.
 */
fun PathSegment.asSingleSegmentPath(): Path = Path(this)

private fun checkContinuity(segments: List<PathSegment>) {
    var prev: PathSegment? = null
    for (cur in segments) {
        if (prev != null) {
            val prevlen = prev.length
            checkCont(prev.position(prevlen), cur.position(0.0), "Position")
            checkCont(prev.positionDeriv(prevlen), cur.positionDeriv(0.0), "PositionDeriv")
            checkCont(
                prev.positionSecondDeriv(prevlen), cur.positionSecondDeriv(0.0), "PositionSecondDeriv"
            )
            //tanAngle, tanAngleDeriv covered by above
//            checkCont(
//                prev.tanAngleSecondDeriv(prevlen), cur.tanAngleSecondDeriv(0.0), "TanAngleSecondDeriv"
//            )

            checkCont(prev.heading(prevlen), cur.heading(0.0), "Heading")
            checkCont(prev.headingDeriv(prevlen), cur.headingDeriv(0.0), "HeadingDeriv")
//            checkCont(prev.headingSecondDeriv(prevlen), cur.headingSecondDeriv(0.0), "HeadingSecondDeriv")
        }
        prev = cur
    }
}


private inline fun checkCont(prev: Vector2d, cur: Vector2d, name: String) {
    require(prev epsEq cur) { "$name discontinuity: $prev, $cur" }
}

private inline fun checkCont(prev: Double, cur: Double, name: String) {
    require(prev epsEq cur) { "$name discontinuity: $prev, $cur" }
}
