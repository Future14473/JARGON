package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.blocks.Block.Processing.IN_FIRST_LAZY
import org.futurerobotics.jargon.blocks.BlocksConfig
import org.futurerobotics.jargon.blocks.CombineBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.matches
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.MotionState


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
 * @param model the [DiscreteLinSSModel] to use
 * @param kGain the kGain matrix
 * @param feedForwardQRCost the [QRCost] to use in calculation feed-forward gain; null if only pseudo-inverse is to be used.
 */
class SSControllerWithFF(
    private val model: DiscreteLinSSModel,
    kGain: Mat,
    feedForwardQRCost: QRCost? = null
) : CombineBlock<MotionState<Vec>, Vec, Vec>(IN_FIRST_LAZY) {

    init {
        require(
            kGain.matches(
                model.inputStructure,
                model.stateStructure
            )
        ) { "kGain must be compatible with this matrix" }
    }

    //flatten model
    private val kGain = kGain.toImmutableMat()
    private val kFF = plantInversionKFF(model, feedForwardQRCost)

    override fun combine(a: MotionState<Vec>, b: Vec): Vec {
        //we don't care about elapsed seconds.
        val r = a.s
        val r1 = a.s + a.v * model.period + a.a * (model.period.squared() / 2)
        val x = b
        return kGain * (r - x) + kFF(r1 - model.A * r)
    }

    /** Reference [BlocksConfig.Input] */
    val reference: BlocksConfig.Input<MotionState<Vec>> get() = firstInput
    /** State [BlocksConfig.Input] */
    val state: BlocksConfig.Input<Vec> get() = secondInput
}