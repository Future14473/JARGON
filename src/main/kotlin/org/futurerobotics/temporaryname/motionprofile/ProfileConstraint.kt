package org.futurerobotics.temporaryname.motionprofile

/**
 * A series of constraints to be used to generate a [MotionProfile] using [MotionProfileGenerator]
 */
interface ProfileConstraint {
    /** Gets the maximum allowable velocity at the point [x] units from the start. */
    fun getMaxVelocity(x: Double): Double

    /** Gets the maximum allowable acceleration at the point [x] units from the start, given [curVelocity].
     *
     * If [reversed] is true, this should return the
     * _negative of the minimum velocity_ instead.
     */
    fun getMaxAccel(x: Double, curVelocity: Double, reversed: Boolean): Double

    /**
     * Gets a Pair of [A list of max point velocities, A list of [MaxAccelGetter]s]
     * corresponding to the points, [allX], as units from the start.
     */
    fun getAllVelsAndAccels(allX: List<Double>): Pair<List<Double>, List<MaxAccelGetter>> =
        allX.map(::getMaxVelocity) to allX.map { x ->
            MaxAccelGetter { curVel, reversed ->
                getMaxAccel(x, curVel, reversed)
            }
        }
}

/**
 * A named function for getting maximum acceleration at a point.
 *
 * Gets maximum acceleration at a particular point, given [curVelocity]
 * @see [ProfileConstraint.getMaxAccel]
 * @see [ProfileConstraint.getAllVelsAndAccels]
 */
interface MaxAccelGetter {
    /**
     * Gets the maximum acceleration at this point given the [curVelocity].
     * If [reversed] is true, this should return the negative of the _minimum_ acceleration instead.
     * @see [ProfileConstraint.getMaxAccel]
     * @see [ProfileConstraint.getAllVelsAndAccels]
     */
    fun getMaxAccel(curVelocity: Double, reversed: Boolean): Double
}

/**
 * Creates a [MaxAccelGetter] from a function. Used for easy creation with lambdas.
 */
@Suppress("FunctionName")
inline fun MaxAccelGetter(crossinline func: (curVelocity: Double, reversed: Boolean) -> Double): MaxAccelGetter =
    object : MaxAccelGetter {
        override fun getMaxAccel(curVelocity: Double, reversed: Boolean): Double {
            return func(curVelocity, reversed)
        }
    }