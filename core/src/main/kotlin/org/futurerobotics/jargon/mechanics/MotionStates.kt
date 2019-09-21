package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d

/**
 * Represents the motion of some quantity representing type [T] (e.g. 1d motion, vector2d, pose, linear algebra vector).
 *
 * This contains velocity ([v]), and acceleration ([a])
 * @see [MotionState3]
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
 * Represents the motion state: current state (s), and derivative (v)
 * of some quantity [T].
 */
interface MotionState2<T : Any> {
    /** The position of this [MotionState3] */
    val s: T
    /** The velocity of this [MotionOnly] */
    val v: T

    /** @return s */
    operator fun component1(): T = s

    /** @return v */
    operator fun component2(): T = v
}

/**
 * Represents the motion state: current state (s), derivative (v), and second derivative [a]
 * of some quantity [T].
 *
 * This is a sub-interface of [MotionState2]
 */
interface MotionState3<T : Any> : MotionState2<T> {
    /** The acceleration of this [MotionState3] */
    val a: T

    /** @return a */
    operator fun component3(): T = a

    /**
     * Creates a [MotionOnly] with same v and a as this [MotionState3]
     */
    fun toMotionOnly(): ValueMotionOnly<T> = ValueMotionOnly(v,a)
}


/** An simple implementation of [MotionOnly] that holds values in fields */
open class ValueMotionOnly<T : Any>(override val v: T, override val a: T) : MotionOnly<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState3<*> -> false
        else -> v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * v.hashCode() + a.hashCode()
}

/** An simple implementation of [MotionOnly] that holds values in fields */
open class ValueMotionState2<T : Any>(override val s: T, override val v: T) : MotionState2<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState3<*> -> false
        else -> s == other.s && v == other.v
    }

    override fun hashCode(): Int = 31 * s.hashCode() + v.hashCode()
}

/** An simple implementation of [MotionState3] that holds values in fields */
open class ValueMotionState3<T : Any>(s: T, v: T, override val a: T) : ValueMotionState2<T>(s, v), MotionState3<T> {
    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState3<*> -> false
        else -> s == other.s && v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * super.hashCode() + s.hashCode()
}


/** Extracts the vector [MotionState3] from this pose [MotionState3]. */
fun MotionState3<Pose2d>.vec(): MotionState3<Vector2d> = ValueMotionState3(s.vec, v.vec, a.vec)

/** Extracts the heading component from this pose [MotionState3]. */
fun MotionState3<Pose2d>.heading(): LinearMotionState3 = LinearMotionState3(s.heading, v.heading, a.heading)

/** Extracts the x component from this pose [MotionState3]. */
fun MotionState3<Pose2d>.x(): LinearMotionState3 = LinearMotionState3(s.x, v.x, a.x)

/** Extracts the y component from this pose [MotionState3]. */
fun MotionState3<Pose2d>.y(): LinearMotionState3 = LinearMotionState3(s.y, v.y, a.y)
