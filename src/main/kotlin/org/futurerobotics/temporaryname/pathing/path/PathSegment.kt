package org.futurerobotics.temporaryname.pathing.path


/**
 * Represents a complete or portion of a path, including heading.
 * This is an extension of a [Curve]
 * @see Curve
 * @see Curve
 * @see Curve
 * Go see it
 */
interface PathSegment : Curve {
    /** The heading (which way the robot, not the path, is facing) [s] units along this path. */
    fun heading(s: Double): Double

    /** The [heading]'s derivative w/rt arc length [s] units along this path. */
    fun headingDeriv(s: Double): Double

    /**
     * The [heading]'s second derivative w/rt arc length [s] units along this path.
     *
     * This may be instantaneously discontinuous.
     */
    fun headingSecondDeriv(s: Double): Double

    /** Gets a [PathPointInfo] for the point [s] units along this path. */
    override fun getPointInfo(s: Double): PathPointInfo = SimplePathPointInfo(this, s)

    /**
     *  Returns a List of [PathPointInfo] for all the points [allS]'s units along the path.
     *
     *  Here is room for some optimizations.
     * @see getPointInfo
     */
    override fun getAllPointInfo(allS: List<Double>): List<PathPointInfo> = allS.map(::getPointInfo)
}
