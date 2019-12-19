package org.futurerobotics.jargon.math

/**
 * Represents just the motion of some quantity of type [T], meaning [vel]ocity and [accel]eration.
 * @see MotionState
 * @see ValueMotionOnly
 * @see LinearMotionOnly
 */
interface MotionOnly<T : Any> {

    /** The velocity of this [MotionOnly] */
    val vel: T
    /** The acceleration of this [MotionOnly] */
    val accel: T

    /** @return v */
    @JvmDefault
    operator fun component1(): T = vel

    /** @return a */
    @JvmDefault
    operator fun component2(): T = accel
}

/**
 * Represents the motion state of some quantity of type [T], meaning [value], [deriv], and [secondDeriv].
 * @see ValueMotionState
 * @see MotionOnly
 * @see LinearMotionState
 */
interface MotionState<T : Any> {

    /** The value of this [MotionState] */
    val value: T
    /** The velocity of this [MotionOnly] */
    val deriv: T
    /** The acceleration of this [MotionState] */
    val secondDeriv: T

    /** @return s */
    @JvmDefault
    operator fun component1(): T = value

    /** @return v */
    @JvmDefault
    operator fun component2(): T = deriv

    /** @return a */
    @JvmDefault
    operator fun component3(): T = secondDeriv

    /**
     * Creates a [MotionOnly] with same v and a as this [MotionState]
     */
    @JvmDefault
    fun toMotionOnly(): ValueMotionOnly<T> =
        ValueMotionOnly(deriv, secondDeriv)
}

/** An simple implementation of [MotionOnly] that holds values in fields */
open class ValueMotionOnly<T : Any>(
    final override val vel: T, final override val accel: T
) : MotionOnly<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> vel == other.deriv && accel == other.secondDeriv
    }

    override fun hashCode(): Int = 31 * vel.hashCode() + accel.hashCode()
}

/** An simple implementation of [MotionState] that holds values in fields */
open class ValueMotionState<T : Any>(
    final override val value: T, final override val deriv: T, final override val secondDeriv: T
) : MotionState<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> value == other.value && deriv == other.deriv && secondDeriv == other.secondDeriv
    }

    override fun hashCode(): Int = 31 * super.hashCode() + value.hashCode()
    override fun toString(): String = "ValueMotionState(s=$value, v=$deriv, a=$secondDeriv)"

    companion object {
        /** Creates a [ValueMotionState] with all s,v,a values equal to [value] */
        @JvmStatic
        fun <T : Any> ofAll(value: T): ValueMotionState<T> =
            ValueMotionState(value, value, value)
    }
}

/** Extracts a vector [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.vec(): MotionState<Vector2d> =
    ValueMotionState(value.vec, deriv.vec, secondDeriv.vec)

/** Extracts a heading [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.heading(): LinearMotionState =
    LinearMotionState(value.heading, deriv.heading, secondDeriv.heading)

/** Extracts a x [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.x(): LinearMotionState =
    LinearMotionState(value.x, deriv.x, secondDeriv.x)

/** Extracts the y [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.y(): LinearMotionState =
    LinearMotionState(value.y, deriv.y, secondDeriv.y)
