@file:Suppress("UNCHECKED_CAST")

package org.futurerobotics.jargon.blocks

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.mechanics.*
import sun.jvm.hotspot.oops.CellTypeState.value

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
class SplitMotionState<T : Any> : ListStoreBlock(1, 3, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionState<T>> {
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
    override val index: Int get() = 0

    /** The value [BlocksConfig.Output] */
    val value: BlocksConfig.Output<T> get() = outputIndex(0)
    /** The velocity [BlocksConfig.Output] */
    val vel: BlocksConfig.Output<T> get() = outputIndex(1)
    /** The acceleration [BlocksConfig.Output] */
    val accel: BlocksConfig.Output<T> get() = outputIndex(2)

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

    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): MotionState<T> = ValueMotionState(
        inputs[0] as T,
        inputs[1] as T,
        inputs[2] as T
    )

    /** The value [BlocksConfig.Input] */
    val value: BlocksConfig.Input<T> get() = inputIndex(0)
    /** The velocity [BlocksConfig.Input] */
    val vel: BlocksConfig.Input<T> get() = inputIndex(1)
    /** The acceleration [BlocksConfig.Input] */
    val accel: BlocksConfig.Input<T> get() = inputIndex(2)

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
class SplitMotionOnly<T : Any> : ListStoreBlock(1, 2, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionOnly<T>> {
    override fun init(outputs: MutableList<Any?>) {}

    @Suppress("UNCHECKED_CAST")
    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        run {
            val t = inputs[0] as MotionOnly<T>
            outputs[0] = t.v
            outputs[1] = t.a
        }
    }

    override val block: Block get() = this
    override val index: Int get() = 0

    /** The velocity [BlocksConfig.Output] */
    val vel: BlocksConfig.Output<T> get() = outputIndex(0)
    /** The acceleration [BlocksConfig.Output] */
    val accel: BlocksConfig.Output<T> get() = outputIndex(1)

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

    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): MotionOnly<T> = ValueMotionOnly(
        inputs[0] as T,
        inputs[1] as T
    )

    /** The velocity [BlocksConfig.Input] */
    val vel: BlocksConfig.Input<T> get() = inputIndex(0)
    /** The acceleration [BlocksConfig.Input] */
    val accel: BlocksConfig.Input<T> get() = inputIndex(1)

    /** [value] */
    operator fun component1(): BlocksConfig.Input<T> = vel

    /** [vel] */
    operator fun component2(): BlocksConfig.Input<T> = accel
}

/**
 * A block that takes in a [Vector2d] and splits it into its parts.
 */
class SplitVector : ListStoreBlock(1, 2, IN_FIRST_LAZY),
    BlocksConfig.Input<Vector2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val (x, y) = inputs[0] as Vector2d
        outputs[0] = x
        outputs[1] = y
    }

    override val block: Block get() = this
    override val index: Int get() = 0

    /** x value [BlocksConfig.Output] */
    val x: BlocksConfig.Output<Double> get() = outputIndex(0)
    /** y value [BlocksConfig.Output] */
    val y: BlocksConfig.Output<Double> get() = outputIndex(1)

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

    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): Vector2d =
        Vector2d(inputs[0] as Double, inputs[1] as Double)

    /** x value [BlocksConfig.Input] */
    val x: BlocksConfig.Input<Double> get() = inputIndex(0)
    /** y value [BlocksConfig.Input] */
    val y: BlocksConfig.Input<Double> get() = inputIndex(1)

    /** [x] */
    operator fun component1(): BlocksConfig.Input<Double> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Input<Double> = y
}


/**
 * A block that takes in a [Pose2d] and splits it into its parts.
 */
class SplitPose : ListStoreBlock(1, 3, IN_FIRST_LAZY),
    BlocksConfig.Input<Pose2d> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val (x, y) = inputs[0] as Pose2d
        outputs[0] = x
        outputs[1] = y
    }

    override val block: Block get() = this
    override val index: Int get() = 0

    /** x value [BlocksConfig.Output] */
    val x: BlocksConfig.Output<Double> get() = outputIndex(0)
    /** y value [BlocksConfig.Output] */
    val y: BlocksConfig.Output<Double> get() = outputIndex(1)
    /** heading value [BlocksConfig.Output] */
    val heading: BlocksConfig.Output<Double> get() = outputIndex(2)

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
class CreatePoseFromComp : SingleOutputBlock<Pose2d>(1, IN_FIRST_LAZY) {
    override fun doInit(): Pose2d? = null

    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Double, inputs[1] as Double, inputs[2] as Double)

    /** x value [BlocksConfig.Input] */
    val x: BlocksConfig.Input<Double> get() = inputIndex(0)
    /** y value [BlocksConfig.Input] */
    val y: BlocksConfig.Input<Double> get() = inputIndex(1)
    /** y value [BlocksConfig.Input] */
    val heading: BlocksConfig.Input<Double> get() = inputIndex(2)

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

    override fun getOutput(inputs: List<Any?>, systemValues: SystemValues): Pose2d =
        Pose2d(inputs[0] as Vector2d, inputs[1] as Double)

    /** x value [BlocksConfig.Input] */
    val vec: BlocksConfig.Input<Vector2d> get() = inputIndex(0)
    /** y value [BlocksConfig.Input] */
    val heading: BlocksConfig.Input<Double> get() = inputIndex(1)

    /** [vec] */
    operator fun component1(): BlocksConfig.Input<Vector2d> = vec

    /** [heading] */
    operator fun component2(): BlocksConfig.Input<Double> = heading

}

/**
 * A block that splits a [MotionState]<[Pose2d]> into 3 [LinearMotionState], each representing a component of [Pose2d]
 */
class SplitPoseMotionState : ListStoreBlock(1, 3, IN_FIRST_LAZY),
    BlocksConfig.Input<MotionState<Pose2d>> {
    override fun init(outputs: MutableList<Any?>) {}

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        val state = inputs[0] as MotionState<Pose2d>
        outputs[0] = state.x()
        outputs[1] = state.y()
        outputs[2] = state.heading()
    }

    /** x MotionState [BlocksConfig.Output] */
    val x: BlocksConfig.Output<LinearMotionState> get() = outputIndex(0)
    /** y MotionState [BlocksConfig.Output] */
    val y: BlocksConfig.Output<LinearMotionState> get() = outputIndex(1)
    /** heading MotionState [BlocksConfig.Output] */
    val heading: BlocksConfig.Output<LinearMotionState> get() = outputIndex(2)

    /** [x] */
    operator fun component1(): BlocksConfig.Output<LinearMotionState> = x

    /** [y] */
    operator fun component2(): BlocksConfig.Output<LinearMotionState> = y

    /** [heading] */
    operator fun component3(): BlocksConfig.Output<LinearMotionState> = heading

    override val block: Block get() = this
    override val index: Int get() = 0
}