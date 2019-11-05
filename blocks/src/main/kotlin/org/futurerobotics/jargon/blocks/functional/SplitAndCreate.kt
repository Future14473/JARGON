@file:Suppress("UNCHECKED_CAST", "MemberVisibilityCanBePrivate")

package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.BaseBlock
import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.SingleInputBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.*

/**
 * A block that takes a [MotionState] and splits it into its components in order.
 */
class SplitMotionState<T : Any> : BaseBlock(LAZY) {

    /** The motion state input */
    val input: Input<MotionState<T>> = newInput()
    /** The value output */
    val value: Output<T> = newOutput()
    /** The velocity output */
    val vel: Output<T> = newOutput()
    /** The acceleration output */
    val accel: Output<T> = newOutput()

    override fun Context.process() {
        val (s, v, a) = input.get
        value.set = s
        vel.set = v
        accel.set = a
    }

    /** [value] */
    operator fun component1(): Output<T> = value

    /** [vel] */
    operator fun component2(): Output<T> = vel

    /** [accel] */
    operator fun component3(): Output<T> = accel
}

/**
 * A block that takes value, velocity, and acceleration, and combines it into a [MotionState].
 */
class CreateMotionState<T : Any> : BaseBlock(LAZY) {

    /** The value input */
    val value: Input<T> = newInput()
    /** The velocity input */
    val vel: Input<T> = newInput()
    /** The acceleration input */
    val accel: Input<T> = newInput()
    /** The motion state output */
    val output: Output<MotionState<T>> = newOutput()

    override fun Context.process() {
        output.set = ValueMotionState(value.get, vel.get, accel.get)
    }

    /** [value] */
    operator fun component1(): Input<T> = value

    /** [vel] */
    operator fun component2(): Input<T> = vel

    /** [accel] */
    operator fun component3(): Input<T> = accel
}

/**
 * A block that takes a [MotionOnly] and splits it into its components in order.
 */
class SplitMotionOnly<T : Any> : BaseBlock(LAZY) {

    /** The motion only input */
    val input: Input<MotionOnly<T>> = newInput()
    /** The velocity output */
    val vel: Output<T> = newOutput()
    /** The acceleration output */
    val accel: Output<T> = newOutput()

    override fun Context.process() {
        val input = input.get
        vel.set = input.v
        accel.set = input.a
    }

    /** [vel] */
    operator fun component1(): Output<T> = vel

    /** [accel] */
    operator fun component2(): Output<T> = accel
}

/**
 * A block that takes velocity and acceleration and combines it into a [MotionOnly]
 */
class CreateMotionOnly<T : Any> : BaseBlock(LAZY) {

    /** The velocity input */
    val vel: Input<T> = newInput()
    /** The acceleration input */
    val accel: Input<T> = newInput()

    /** The motion only output */
    val output: Output<MotionOnly<T>> = newOutput()

    override fun Context.process() {
        val vel = vel.get
        val accel = accel.get
        output.set = ValueMotionOnly(vel, accel)
    }

    /** [vel] */
    operator fun component1(): Input<T> = vel

    /** [accel] */
    operator fun component2(): Input<T> = accel
}

/**
 * Creates a [Pose2d] from x, y, heading components.
 */
class SplitPose : SingleInputBlock<Pose2d>(LAZY) {

    /** x */
    val x: Output<Double> = newOutput()
    /** y */
    val y: Output<Double> = newOutput()
    /** heading */
    val heading: Output<Double> = newOutput()

    override fun Context.process(input: Pose2d) {
        x.set = input.x
        y.set = input.y
        heading.set = input.heading
    }
}

/**
 * Splits a [Pose2d] into x, y, heading components.
 */
class CreatePose : SingleOutputBlock<Pose2d>(LAZY) {

    /** x */
    val x: Input<Double> = newInput()
    /** y */
    val y: Input<Double> = newInput()
    /** heading */
    val heading: Input<Double> = newInput()

    override fun Context.getOutput(): Pose2d = Pose2d(x.get, y.get, heading.get)
}

/**
 * A block that splits a [MotionState]<[Pose2d]> into 3 [LinearMotionState]s, each representing a component of [Pose2d]
 */
class SplitPoseMotionState : BaseBlock(LAZY) {

    /** The motion state input */
    val input: Input<MotionState<Pose2d>> = newInput()
    /** x MotionState output */
    val x: Output<LinearMotionState> = newOutput()
    /** y MotionState output */
    val y: Output<LinearMotionState> = newOutput()
    /** heading MotionState output */
    val heading: Output<LinearMotionState> = newOutput()

    override fun Context.process() {
        val input = input.get
        x.set = input.x()
        y.set = input.y()
        heading.set = input.heading()
    }

    /** [x] */
    operator fun component1(): Output<LinearMotionState> = x

    /** [y] */
    operator fun component2(): Output<LinearMotionState> = y

    /** [heading] */
    operator fun component3(): Output<LinearMotionState> = heading
}
