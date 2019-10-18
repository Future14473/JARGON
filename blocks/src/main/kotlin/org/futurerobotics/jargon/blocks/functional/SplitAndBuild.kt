@file:Suppress("UNCHECKED_CAST", "MemberVisibilityCanBePrivate")

package org.futurerobotics.jargon.blocks.functional

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.SingleInputListStoreBlock
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.blocks.SystemValues
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.mechanics.*

/**
 * A block that takes a [MotionState] and splits it into its components in order.
 *
 * Inputs:
 * 1. A [MotionState] of [T].
 *
 * Outputs:
 * 1. value
 * 2. velocity
 * 3. acceleration
 */
class SplitMotionState<T : Any> : SingleInputListStoreBlock<MotionState<T>>(3, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionState<T>> {
    override fun init(outputs: MutableList<Any?>) {}

    @Suppress("UNCHECKED_CAST")
    override fun processInput(
        input: MotionState<T>,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ) {
        outputs[0] = input.s
        outputs[1] = input.v
        outputs[2] = input.a
    }

    /** The value [BlocksConfig.Output] */
    val value: BlocksConfig.Output<T> get() = configOutput(0)
    /** The velocity [BlocksConfig.Output] */
    val vel: BlocksConfig.Output<T> get() = configOutput(1)
    /** The acceleration [BlocksConfig.Output] */
    val accel: BlocksConfig.Output<T> get() = configOutput(2)

    /** [value] */
    operator fun component1(): BlocksConfig.Output<T> = value

    /** [vel] */
    operator fun component2(): BlocksConfig.Output<T> = vel

    /** [accel] */
    operator fun component3(): BlocksConfig.Output<T> = accel
}

/**
 * A block that takes the value, velocity, and acceleration and combines it into a [MotionState]
 *
 * Inputs:
 *
 * 1. value
 * 2. velocity
 * 3. acceleration
 *
 * Outputs:
 * 1. A [MotionState] of [T].
 */
class CreateMotionState<T : Any> : SingleOutputBlock<MotionState<T>>(3, IN_FIRST_LAZY) {
    override fun doInit(): MotionState<T>? = null

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): MotionState<T> = ValueMotionState(
        inputs[0] as T,
        inputs[1] as T,
        inputs[2] as T
    )

    /** The value [BlocksConfig.Input] */
    val value: BlocksConfig.Input<T> get() = configInput(0)
    /** The velocity [BlocksConfig.Input] */
    val vel: BlocksConfig.Input<T> get() = configInput(1)
    /** The acceleration [BlocksConfig.Input] */
    val accel: BlocksConfig.Input<T> get() = configInput(2)

    /** [value] */
    operator fun component1(): BlocksConfig.Input<T> = value

    /** [vel] */
    operator fun component2(): BlocksConfig.Input<T> = vel

    /** [accel] */
    operator fun component3(): BlocksConfig.Input<T> = accel
}


/**
 * A block that takes a [MotionState] and splits it into its components in order.
 *
 * Inputs:
 * 1. A [MotionState] of [T].
 *
 * Outputs:
 * 1. value
 * 2. velocity
 * 3. acceleration
 */
class SplitMotionOnly<T : Any> : SingleInputListStoreBlock<MotionOnly<T>>(2, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionOnly<T>> {
    override fun init(outputs: MutableList<Any?>) {}


    override fun processInput(
        input: MotionOnly<T>,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ) {
        outputs[0] = input.v
        outputs[1] = input.a
    }

    /** The velocity [BlocksConfig.Output] */
    val vel: BlocksConfig.Output<T> get() = configOutput(0)
    /** The acceleration [BlocksConfig.Output] */
    val accel: BlocksConfig.Output<T> get() = configOutput(1)

    /** [vel] */
    operator fun component1(): BlocksConfig.Output<T> = vel

    /** [accel] */
    operator fun component2(): BlocksConfig.Output<T> = accel

}

/**
 * A block that takes the value, velocity, and acceleration and combines it into a [MotionState]
 *
 * Inputs:
 *
 * 1. value
 * 2. velocity
 * 3. acceleration
 *
 * Outputs:
 * 1. A [MotionState] of [T].
 */
class CreateMotionOnly<T : Any> : SingleOutputBlock<MotionOnly<T>>(2, IN_FIRST_LAZY) {
    override fun doInit(): MotionOnly<T>? = null

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): MotionOnly<T> =
        ValueMotionOnly(inputs[0] as T, inputs[1] as T)

    /** The velocity [BlocksConfig.Input] */
    val vel: BlocksConfig.Input<T> get() = configInput(0)
    /** The acceleration [BlocksConfig.Input] */
    val accel: BlocksConfig.Input<T> get() = configInput(1)

    /** [value] */
    operator fun component1(): BlocksConfig.Input<T> = vel

    /** [vel] */
    operator fun component2(): BlocksConfig.Input<T> = accel
}

/**
 * A block that takes in a [Vector2d] and splits it into its parts.
 */
class SplitVector : SingleInputListStoreBlock<Vector2d>(2, IN_FIRST_LAZY),
    BlocksConfig.Input<Vector2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun processInput(
        input: Vector2d,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x
        outputs[1] = input.y
    }

    /** x value [BlocksConfig.Output] */
    val x: BlocksConfig.Output<Double> get() = configOutput(0)
    /** y value [BlocksConfig.Output] */
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
    override fun doInit(): Vector2d? = null

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Vector2d =
        Vector2d(inputs[0] as Double, inputs[1] as Double)

    /** x value [BlocksConfig.Input] */
    val x: BlocksConfig.Input<Double> get() = configInput(0)
    /** y value [BlocksConfig.Input] */
    val y: BlocksConfig.Input<Double> get() = configInput(1)

    /** [x] */
    operator fun component1(): BlocksConfig.Input<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Input<Double> = y
}


/**
 * A block that takes in a [Pose2d] and splits it into its parts.
 */
class SplitPose : SingleInputListStoreBlock<Pose2d>(3, IN_FIRST_LAZY),
    BlocksConfig.Input<Pose2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun processInput(
        input: Pose2d,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x
        outputs[1] = input.y
        outputs[2] = input.heading
    }

    /** x value [BlocksConfig.Output] */
    val x: BlocksConfig.Output<Double> get() = configOutput(0)
    /** y value [BlocksConfig.Output] */
    val y: BlocksConfig.Output<Double> get() = configOutput(1)
    /** heading value [BlocksConfig.Output] */
    val heading: BlocksConfig.Output<Double> get() = configOutput(2)

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
    override fun doInit(): Pose2d? = null

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Double, inputs[1] as Double, inputs[2] as Double)

    /** x value [BlocksConfig.Input] */
    val x: BlocksConfig.Input<Double> get() = configInput(0)
    /** y value [BlocksConfig.Input] */
    val y: BlocksConfig.Input<Double> get() = configInput(1)
    /** y value [BlocksConfig.Input] */
    val heading: BlocksConfig.Input<Double> get() = configInput(2)

    /** [x] */
    operator fun component1(): BlocksConfig.Input<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Input<Double> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Input<Double> = heading
}


/**
 * A block that creates a pose from x, y, and heading components.
 */
class CreatePoseFromVec : SingleOutputBlock<Pose2d>(1, IN_FIRST_LAZY) {
    override fun doInit(): Pose2d? = null

    override fun processOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Vector2d, inputs[1] as Double)

    /** x value [BlocksConfig.Input] */
    val vec: BlocksConfig.Input<Vector2d> get() = configInput(0)
    /** y value [BlocksConfig.Input] */
    val heading: BlocksConfig.Input<Double> get() = configInput(1)

    /** [vec] */
    operator fun component1(): BlocksConfig.Input<Vector2d> = vec

    /** [heading] */
    operator fun component2(): BlocksConfig.Input<Double> = heading

}

/**
 * A block that splits a [MotionState]<[Pose2d]> into 3 [LinearMotionState], each representing a component of [Pose2d]
 */
class SplitPoseMotionState : SingleInputListStoreBlock<MotionState<Pose2d>>(3, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionState<Pose2d>> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun processInput(
        input: MotionState<Pose2d>,
        systemValues: SystemValues,
        outputs: MutableList<Any?>
    ) {
        outputs[0] = input.x()
        outputs[1] = input.y()
        outputs[2] = input.heading()
    }

    /** x MotionState [BlocksConfig.Output] */
    val x: BlocksConfig.Output<LinearMotionState> get() = configOutput(0)
    /** y MotionState [BlocksConfig.Output] */
    val y: BlocksConfig.Output<LinearMotionState> get() = configOutput(1)
    /** heading MotionState [BlocksConfig.Output] */
    val heading: BlocksConfig.Output<LinearMotionState> get() = configOutput(2)

    /** [x] */
    operator fun component1(): BlocksConfig.Output<LinearMotionState> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Output<LinearMotionState> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Output<LinearMotionState> = heading
}