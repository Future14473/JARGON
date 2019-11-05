package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.blocks.Block.Processing.LAZY
import org.futurerobotics.jargon.blocks.SingleOutputBlock
import org.futurerobotics.jargon.blocks.control.Controller
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.mechanics.MotionState
import kotlin.math.pow

/**
 * A state space controller using the given [model] and [kGain], with feed forward.
 *
 * Inputs:
 * 1. Either: [MotionState] of the reference vector
 * 2. Current state vector
 *
 * Outputs:
 * 1. Signal vector
 *
 * @param model the [DiscreteLinearStateSpaceModel] to use
 * @param kGain the kGain matrix
 * @param ffQRCost the [QRCost] to use in calculation feed-forward gain; null if only pseudo-inverse is to be used.
 * @param recalculateFeedForward if true, recalculates feed forward every time the loop is run. Use when you know the
 *                               model might change. Note: if you want to recalculate kGain, do so elsewhere and
 *                               modify since we have no way of knowing how you might do this.
 */
class SSControllerWithFF @JvmOverloads constructor(
    private val model: DiscreteLinearStateSpaceModel,
    private val kGain: Mat,
    private val ffQRCost: QRCost? = null,
    private val recalculateFeedForward: Boolean = false
) : SingleOutputBlock<Vec>(LAZY), Controller<MotionState<Vec>, Vec, Vec> {

    override val reference: Input<MotionState<Vec>> = newInput()
    override val state: Input<Vec> = newInput()
    override val signal: Output<Vec> get() = super.output

    init {
        require(kGain.rows == model.inputSize && kGain.cols == model.stateSize) {
            "kGain size (${kGain.rows} x ${kGain.cols}) must be compatible with this model"
        }
    }

    //flatten model
    private val kFF: Mat? = if (recalculateFeedForward) null else plantInversionKFF(model, ffQRCost)

    override fun Context.getOutput(): Vec {
        val ref = reference.get
        val x = state.get
        val r = ref.s
        val r1 = ref.s + ref.v * model.period + ref.a * (model.period.pow(2) / 2)
        val kFF = kFF ?: plantInversionKFF(
            model,
            ffQRCost
        )
        return kGain * (r - x) + kFF(r1 - model.A * r)
    }
}
