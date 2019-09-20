package org.futurerobotics.jargon.mechanics

import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d

/**
 * Represents the motion of some quantity representing type [T] (e.g. 1d motion, vector2d, pose, linear algebra vector).
 *
 * This contains velocity ([v]), and acceleration ([a])
 * @see [State]
 */
interface Motion<T : Any> {

    /** The velocity of this [Motion] */
    val v: T
    /** The acceleration of this [Motion] */
    val a: T
}

/** @return v */
operator fun <T : Any> Motion<T>.component1(): T = v

/** @return a */
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
interface State<T : Any> : Motion<T> {

    /** The position of this [State] */
    val s: T
    /** The velocity of this [State] */
    override val v: T
    /** The acceleration of this [State] */
    override val a: T
}

/** @return s */
operator fun <T : Any> State<T>.component1(): T = s

/** @return v */
operator fun <T : Any> State<T>.component2(): T = v

/** @return a */
operator fun <T : Any> State<T>.component3(): T = a

/** An simple implementation of [Motion] that holds values in fields */
open class ValueMotion<T : Any>(override val v: T, override val a: T) : Motion<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueState<*> -> false
        else -> v == other.v && a == other.a
    }

    override fun hashCode(): Int = 31 * v.hashCode() + a.hashCode()
}

/** An simple implementation of [State] that holds values in fields */
open class ValueState<T : Any>(override val s: T, v: T, a: T) : ValueMotion<T>(v, a), State<T> {

    override fun equals(other: Any?): Boolean = when {
        this === other -> true
        other !is ValueState<*> -> false
        else -> super.equals(other) && s == other.s
    }

    override fun hashCode(): Int = 31 * super.hashCode() + s.hashCode()
}


/** Extracts the vector [State] from this pose [State]. */
fun State<Pose2d>.vec(): State<Vector2d> = ValueState(s.vec, v.vec, a.vec)

/** Extracts the heading component from this pose [State]. */
fun State<Pose2d>.heading(): LinearState = LinearState(s.heading, v.heading, a.heading)

/** Extracts the x component from this pose [State]. */
fun State<Pose2d>.x(): LinearState = LinearState(s.x, v.x, a.x)

/** Extracts the y component from this pose [State]. */
fun State<Pose2d>.y(): LinearState = LinearState(s.y, v.y, a.y)