@file:Suppress("UNCHECKED_CAST", "MemberVisibilityCanBePrivate")

package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.*
import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.mechanics.*

/**
 * A block that takes a [MotionState] and splits it into its components in order.
 */
class SplitMotionState<T : Any> : SingleInputBlock<MotionState<T>>(3, IN_FIRST_LAZY),
                                  BlocksConfig.Input<MotionState<T>> {

    /** The value output */
    val value: BlocksConfig.Output<T> get() = configOutput(0)
    /** The velocity output */
    val vel: BlocksConfig.Output<T> get() = configOutput(1)
    /** The acceleration output */
    val accel: BlocksConfig.Output<T> get() = configOutput(2)

    override fun init() {
    }

    override fun processInput(input: MotionState<T>, systemValues: SystemValues) {
    }

    override fun getOutput(input: MotionState<T>, index: Int): Any? = when (index) {
        0 -> input.s
        1 -> input.v
        2 -> input.a
        else -> throw IndexOutOfBoundsException(index)
    }

    /** [value] */
    operator fun component1(): BlocksConfig.Output<T> = value

    /** [vel] */
    operator fun component2(): BlocksConfig.Output<T> = vel

    /** [accel] */
    operator fun component3(): BlocksConfig.Output<T> = accel
}

/**
 * A block that takes value, velocity, and acceleration, and combines it into a [MotionState].
 */
class CreateMotionState<T : Any> : SingleOutputBlock<MotionState<T>>(3, IN_FIRST_LAZY) {

    override fun initialValue(): MotionState<T>? = null
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): MotionState<T> =
        ValueMotionState(inputs[0] as T, inputs[1] as T, inputs[2] as T)

    /** The value input */
    val value: BlocksConfig.Input<T> get() = configInput(0)
    /** The velocity input */
    val vel: BlocksConfig.Input<T> get() = configInput(1)
    /** The acceleration input */
    val accel: BlocksConfig.Input<T> get() = configInput(2)

    /** [value] */
    operator fun component1(): BlocksConfig.Input<T> = value

    /** [vel] */
    operator fun component2(): BlocksConfig.Input<T> = vel

    /** [accel] */
    operator fun component3(): BlocksConfig.Input<T> = accel
}

/**
 * A block that takes a [MotionOnly] and splits it into its components in order.
 */
class SplitMotionOnly<T : Any> : SingleInputListStoreBlock<MotionOnly<T>>(2, IN_FIRST_LAZY),
                                 BlocksConfig.Input<MotionOnly<T>> {

    override fun init(outputs: MutableList<Any?>) {}
    /** The velocity output */
    val vel: BlocksConfig.Output<T> get() = configOutput(0)
    /** The acceleration output */
    val accel: BlocksConfig.Output<T> get() = configOutput(1)

    override fun processInput(
        input: MotionOnly<T>, systemValues: SystemValues, outputs: MutableList<Any?>
    ) {
        outputs[0] = input.v
        outputs[1] = input.a
    }

    /** [vel] */
    operator fun component1(): BlocksConfig.Output<T> = vel

    /** [accel] */
    operator fun component2(): BlocksConfig.Output<T> = accel
}

/**
 * A block that takes velocity and acceleration and combines it into a [MotionOnly]
 */
class CreateMotionOnly<T : Any> : CombineBlock<T, T, MotionOnly<T>>(IN_FIRST_LAZY) {

    override fun combine(a: T, b: T): MotionOnly<T> = ValueMotionOnly(a, b)
    /** The velocity input */
    val vel: BlocksConfig.Input<T> get() = firstInput
    /** The acceleration input */
    val accel: BlocksConfig.Input<T> get() = secondInput

    /** [value] */
    operator fun component1(): BlocksConfig.Input<T> = vel

    /** [vel] */
    operator fun component2(): BlocksConfig.Input<T> = accel
}

/**
 * A block that takes in a [Vector2d] and splits it into x and y components.
 */
class SplitVector : SingleInputListStoreBlock<Vector2d>(2, IN_FIRST_LAZY), BlocksConfig.Input<Vector2d> {

    override fun init(outputs: MutableList<Any?>) {}
    override fun processInput(
        input: Vector2d, systemValues: SystemValues, outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x
        outputs[1] = input.y
    }

    /** x value output */
    val x: BlocksConfig.Output<Double> get() = configOutput(0)
    /** y value output */
    val y: BlocksConfig.Output<Double> get() = configOutput(1)

    /** [x] */
    operator fun component1(): BlocksConfig.Output<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Output<Double> = y
}

/**
 * A block that creates a vector from x and y components.
 */
class CreateVector : SingleOutputBlock<Vector2d>(1, IN_FIRST_LAZY) {

    override fun initialValue(): Vector2d? = null
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vector2d =
        Vector2d(inputs[0] as Double, inputs[1] as Double)

    /** x value input */
    val x: BlocksConfig.Input<Double> get() = configInput(0)
    /** y value input */
    val y: BlocksConfig.Input<Double> get() = configInput(1)

    /** [x] */
    operator fun component1(): BlocksConfig.Input<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Input<Double> = y
}

/**
 * A block that takes in a [Pose2d] and splits it into x, y, and heading components.
 */
class SplitPose : SingleInputListStoreBlock<Pose2d>(3, IN_FIRST_LAZY), BlocksConfig.Input<Pose2d> {

    override fun init(outputs: MutableList<Any?>) {}
    /** x value output */
    val x: BlocksConfig.Output<Double> get() = configOutput(0)
    /** y value output */
    val y: BlocksConfig.Output<Double> get() = configOutput(1)
    /** heading value output */
    val heading: BlocksConfig.Output<Double> get() = configOutput(2)

    override fun processInput(
        input: Pose2d, systemValues: SystemValues, outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x
        outputs[1] = input.y
        outputs[2] = input.heading
    }

    /** [x] */
    operator fun component1(): BlocksConfig.Output<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Output<Double> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Output<Double> = heading
}

/**
 * A block that creates a pose from x, y, and heading components.
 */
class CreatePoseFromComp : SingleOutputBlock<Pose2d>(3, IN_FIRST_LAZY) {

    override fun initialValue(): Pose2d? = null
    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Double, inputs[1] as Double, inputs[2] as Double)

    /** x value input */
    val x: BlocksConfig.Input<Double> get() = configInput(0)
    /** y value input */
    val y: BlocksConfig.Input<Double> get() = configInput(1)
    /** y value input */
    val heading: BlocksConfig.Input<Double> get() = configInput(2)

    /** [x] */
    operator fun component1(): BlocksConfig.Input<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Input<Double> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Input<Double> = heading
}

/**
 * A block that creates a pose from vector and heading components.
 */
class CreatePoseFromVec : SingleOutputBlock<Pose2d>(1, IN_FIRST_LAZY) {

    override fun initialValue(): Pose2d? = null
    /** x value input */
    val vec: BlocksConfig.Input<Vector2d> get() = configInput(0)
    /** y value input */
    val heading: BlocksConfig.Input<Double> get() = configInput(1)

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Vector2d, inputs[1] as Double)

    /** [vec] */
    operator fun component1(): BlocksConfig.Input<Vector2d> = vec

    /** [heading] */
    operator fun component2(): BlocksConfig.Input<Double> = heading
}

/**
 * A block that splits a [MotionState]<[Pose2d]> into 3 [LinearMotionState]s, each representing a component of [Pose2d]
 */
class SplitPoseMotionState : SingleInputListStoreBlock<MotionState<Pose2d>>(3, IN_FIRST_LAZY),
                             BlocksConfig.Input<MotionState<Pose2d>> {

    override fun init(outputs: MutableList<Any?>) {}
    override fun processInput(
        input: MotionState<Pose2d>, systemValues: SystemValues, outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x()
        outputs[1] = input.y()
        outputs[2] = input.heading()
    }

    /** x MotionState output */
    val x: BlocksConfig.Output<LinearMotionState> get() = configOutput(0)
    /** y MotionState output */
    val y: BlocksConfig.Output<LinearMotionState> get() = configOutput(1)
    /** heading MotionState output */
    val heading: BlocksConfig.Output<LinearMotionState> get() = configOutput(2)

    /** [x] */
    operator fun component1(): BlocksConfig.Output<LinearMotionState> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Output<LinearMotionState> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Output<LinearMotionState> = heading
}
