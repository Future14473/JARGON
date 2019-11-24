package org.futurerobotics.jargon.blocks.control

import org.futurerobotics.jargon.blocks.Block
import org.futurerobotics.jargon.blocks.IllegalBlockConfigurationException
import org.futurerobotics.jargon.blocks.ReadOnlyBlockArrangement
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.MotionState
import org.futurerobotics.jargon.statespace.StateSpaceRunner

/**
 * A block that runs a [StateSpaceRunner].
 *
 * An `initialState` can to be provided. If it is null, [stateOverride] **must** be connected
 * and provide an initial state on the first cycle.
 *
 * Either [reference] and [nextReference] can be connected, or [referenceMotionState] can be connected.
 * The former provides reference tracking (if an appropriate [FeedForwardWrapper] is given) in discrete time,
 * while the latter approximates it from continuous time.
 */
class StateSpaceRunnerBlock(private val runner: StateSpaceRunner, private val initialState: Vec?) :
    Block(Processing.ALWAYS) {

    /** The reference vector input. Either this or [referenceMotionState] must be connected. */
    val reference: Input<Vec?> = newOptionalInput()
    /** The _next_ reference vector input, if known. Only used if [reference] is connected, not [referenceMotionState]. */
    val nextReference: Input<Vec?> = newOptionalInput()

    /** The reference motion state. Only used if [reference] is not connected. */
    val referenceMotionState: Input<MotionState<Vec>?> = newOptionalInput()

    /** The measurement input. */
    val measurement: Input<Vec> = newInput()
    /** The state override input; if not null, will override the observer's state. */
    val stateOverride: Input<Vec?> = newOptionalInput()

    /** The resulting signal. */
    val signal: Output<Vec> = newOutput()
    /** The current estimated state. */
    val currentState: Output<Vec> = newOutput()

    private var useMotionState: Boolean = false

    override fun checkConfig(arrangement: ReadOnlyBlockArrangement) {
        val r = reference in arrangement
        val rs = referenceMotionState in arrangement
        if (!r && !rs) {
            throw IllegalBlockConfigurationException("Either reference or referenceState must be connected.")
        }
        if (r && rs) {
            throw IllegalBlockConfigurationException("Both reference or referenceState are connected.")
        }
        useMotionState = rs
    }

    override fun init() {
        if (initialState != null) {
            runner.reset(initialState)
        }
    }

    override fun Context.process() {
        val r: Vec
        val r1: Vec?
        val period = runner.period
        if (useMotionState) {
            val (s, v, a) = referenceMotionState.get!!
            r = s
            r1 = s + v * period + (period * period / 2) * a
        } else {
            r = reference.get!!
            r1 = nextReference.get
        }

        val y = measurement.get
        val stateOverride = stateOverride.get
        if (stateOverride != null) runner.currentState = stateOverride

        runner.update(y, r, r1, totalTimeInNanos)

        signal.set = runner.signal
        currentState.set = runner.currentState
    }
}
