@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.control.Block.Processing.IN_FIRST_LAZY
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
class SplitMotionState<T : Any> : ListStoreBlock(1, 3, IN_FIRST_LAZY), BlockInput<MotionState<T>> {
    override fun init(outputs: MutableList<Any?>) {}

    @Suppress("UNCHECKED_CAST")
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        run {
            val t = inputs[0] as MotionState<T>
            outputs[0] = t.s
            outputs[1] = t.v
            outputs[2] = t.a
        }
    }

    override val block: Block get() = this
    override val inputIndex: Int get() = 0

    /** The value [BlockOutput] */
    val value: BlockOutput<T> get() = outputIndex(0)
    /** The velocity [BlockOutput] */
    val vel: BlockOutput<T> get() = outputIndex(1)
    /** The acceleration [BlockOutput] */
    val accel: BlockOutput<T> get() = outputIndex(2)
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

    override fun getOutput(inputs: List<Any?>): MotionState<T> {
        return ValueMotionState(
            inputs[0] as T,
            inputs[1] as T,
            inputs[2] as T
        )
    }

    /** The value [BlockInput] */
    val value: BlockInput<T> get() = inputIndex(0)
    /** The velocity [BlockInput] */
    val vel: BlockInput<T> get() = inputIndex(1)
    /** The acceleration [BlockInput] */
    val accel: BlockInput<T> get() = inputIndex(2)
}

/**
 * A block that takes in a [Vector2d] and splits it into its parts.
 */
class SplitVector : ListStoreBlock(1, 2, IN_FIRST_LAZY), BlockInput<Vector2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val (x, y) = inputs[0] as Vector2d
        outputs[0] = x
        outputs[1] = y
    }

    override val block: Block get() = this
    override val inputIndex: Int get() = 0

    /** x value [BlockOutput] */
    val x: BlockOutput<Double> get() = outputIndex(0)
    /** y value [BlockOutput] */
    val y: BlockOutput<Double> get() = outputIndex(1)
}

/**
 * A block that creates a vector from x and y components.
 */
class CreateVector : SingleOutputBlock<Vector2d>(1, IN_FIRST_LAZY) {
    override fun doInit(): Vector2d? = null

    override fun getOutput(inputs: List<Any?>): Vector2d = Vector2d(inputs[0] as Double, inputs[1] as Double)

    /** x value [BlockInput] */
    val x: BlockInput<Double> get() = inputIndex(0)
    /** y value [BlockInput] */
    val y: BlockInput<Double> get() = inputIndex(1)
}


/**
 * A block that takes in a [Pose2d] and splits it into its parts.
 */
class SplitPose : ListStoreBlock(1, 3, IN_FIRST_LAZY), BlockInput<Pose2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val (x, y) = inputs[0] as Pose2d
        outputs[0] = x
        outputs[1] = y
    }

    override val block: Block get() = this
    override val inputIndex: Int get() = 0

    /** x value [BlockOutput] */
    val x: BlockOutput<Double> get() = outputIndex(0)
    /** y value [BlockOutput] */
    val y: BlockOutput<Double> get() = outputIndex(1)
    /** heading value [BlockOutput] */
    val heading: BlockOutput<Double> get() = outputIndex(2)
}

/**
 * A block that creates a pose from x, y, and heading components.
 */
class CreatePoseFromComp : SingleOutputBlock<Pose2d>(1, IN_FIRST_LAZY) {
    override fun doInit(): Pose2d? = null

    override fun getOutput(inputs: List<Any?>): Pose2d =
        Pose2d(inputs[0] as Double, inputs[1] as Double, inputs[2] as Double)

    /** x value [BlockInput] */
    val x: BlockInput<Double> get() = inputIndex(0)
    /** y value [BlockInput] */
    val y: BlockInput<Double> get() = inputIndex(1)
    /** y value [BlockInput] */
    val heading: BlockInput<Double> get() = inputIndex(2)
}


/**
 * A block that creates a pose from x, y, and heading components.
 */
class CreatePoseFromVec : SingleOutputBlock<Pose2d>(1, IN_FIRST_LAZY) {
    override fun doInit(): Pose2d? = null

    override fun getOutput(inputs: List<Any?>): Pose2d = Pose2d(inputs[0] as Vector2d, inputs[1] as Double)

    /** x value [BlockInput] */
    val vec: BlockInput<Vector2d> get() = inputIndex(0)
    /** y value [BlockInput] */
    val heading: BlockInput<Double> get() = inputIndex(1)
}

/**
 * A block that splits a [MotionState]<[Pose2d]> into 3 [LinearMotionState], each representing a component of [Pose2d]
 */
class SplitPoseMotionState : ListStoreBlock(1, 3, IN_FIRST_LAZY), BlockInput<MotionState<Pose2d>> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val state = inputs[0] as MotionState<Pose2d>
        outputs[0] = state.x()
        outputs[1] = state.y()
        outputs[2] = state.heading()
    }

    /** x MotionState [BlockOutput] */
    val x: BlockOutput<LinearMotionState> get() = outputIndex(0)
    /** y MotionState [BlockOutput] */
    val y: BlockOutput<LinearMotionState> get() = outputIndex(1)
    /** heading MotionState [BlockOutput] */
    val heading: BlockOutput<LinearMotionState> get() = outputIndex(2)

    override val block: Block get() = this
    override val inputIndex: Int get() = 0
}