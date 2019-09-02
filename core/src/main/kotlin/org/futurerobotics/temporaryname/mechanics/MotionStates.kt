package org.futurerobotics.temporaryname.mechanics

/**
 * Represents the motion of some quantity representing type [T] (e.g. 1d motion, vector2d, pose, linear algebra vector).
 *
 * This contains velocity ([v]), and acceleration ([a])
 * @see [MotionState]
 */
interface Motion<T : Any> {

    /** The velocity of this [Motion] */
    val v: T
    /** The acceleration of this [Motion] */
    val a: T
}

operator fun <T : Any> Motion<T>.component1(): T = v
operator fun <T : Any> Motion<T>.component2(): T = a

/**
 * Represents the motion state _including position/current state_
 * of some space or quantity representing type [T] (e.g. 1d motion, vector, pose, matrix).
 *
 * This contains position ([s]), velocity ([v]), and acceleration ([a])
 *
 * This is a subclass of:
 * @see [Motion]
 */
interface MotionState<T : Any> : Motion<T> {

    /** The position of this [MotionState] */
    val s: T
    /** The velocity of this [MotionState] */
    override val v: T
    /** The acceleration of this [MotionState] */
    override val a: T
}

operator fun <T : Any> MotionState<T>.component1(): T = s
operator fun <T : Any> MotionState<T>.component2(): T = v
operator fun <T : Any> MotionState<T>.component3(): T = a

/** An simple implementation of [Motion] that holds values in fields */
open class ValueMotion<T : Any>(override val v: T, override val a: T) : Motion<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * v.hashCode() + a.hashCode()
}

/** An simple implementation of [MotionState] that holds values in fields */
open class ValueMotionState<T : Any>(override val s: T, v: T, a: T) : ValueMotion<T>(v, a), MotionState<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueMotionState<*> -> false
        else -> super.equals(other) && s == other.s
    }

    override fun hashCode(): Int = 31 * super.hashCode() + s.hashCode()
}

