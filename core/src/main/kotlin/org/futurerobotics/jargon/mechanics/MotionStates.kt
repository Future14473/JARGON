package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d

/**
 * Represents the motion of some quantity representing type [T] (e.g. 1d motion, vector2d, pose, linear algebra vector).
 *
 * This contains velocity ([v]), and acceleration ([a])
 * @see [MotionState]
 */
interface MotionOnly<T : Any> {
    /** The velocity of this [MotionOnly] */
    val v: T
    /** The acceleration of this [MotionOnly] */
    val a: T

    /** @return v */
    operator fun component1(): T = v

    /** @return a */
    operator fun component2(): T = a
}

/**
 * Represents the motion state: current state (s), derivative (v), and second derivative [a]
 * of some quantity [T].
 */
interface MotionState<T : Any> {
    /** The position of this [MotionState] */
    val s: T
    /** The velocity of this [MotionOnly] */
    val v: T

    /** @return s */
    operator fun component1(): T = s

    /** @return v */
    operator fun component2(): T = v

    /** The acceleration of this [MotionState] */
    val a: T

    /** @return a */
    operator fun component3(): T = a

    /**
     * Creates a [MotionOnly] with same v and a as this [MotionState]
     */
    fun toMotionOnly(): ValueMotionOnly<T> = ValueMotionOnly(v, a)
}


/** An simple implementation of [MotionOnly] that holds values in fields */
open class ValueMotionOnly<T : Any>(override val v: T, override val a: T) : MotionOnly<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * v.hashCode() + a.hashCode()
}

/** An simple implementation of [MotionState] that holds values in fields */
open class ValueMotionState<T : Any>(override val s: T, override val v: T, override val a: T) : MotionState<T> {
    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> s == other.s && v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * super.hashCode() + s.hashCode()
    override fun toString(): String = "ValueMotionState(s=$s, v=$v, a=$a)"

    companion object {
        /** Creates a [ValueMotionState] with all s,v,a values equal to [value] */
        @JvmStatic
        fun <T : Any> ofAll(value: T): ValueMotionState<T> = ValueMotionState(value, value, value)
    }
}


/** Extracts the vector [MotionState] from this pose [MotionState]. */
fun MotionState<Pose2d>.vec(): MotionState<Vector2d> = ValueMotionState(s.vec, v.vec, a.vec)

/** Extracts the heading component from this pose [MotionState]. */
fun MotionState<Pose2d>.heading(): LinearMotionState = LinearMotionState(s.heading, v.heading, a.heading)

/** Extracts the x component from this pose [MotionState]. */
fun MotionState<Pose2d>.x(): LinearMotionState = LinearMotionState(s.x, v.x, a.x)

/** Extracts the y component from this pose [MotionState]. */
fun MotionState<Pose2d>.y(): LinearMotionState = LinearMotionState(s.y, v.y, a.y)
