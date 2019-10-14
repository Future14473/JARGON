package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.blocks.Combine
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.matches
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.MotionState


/**
 * A state space controller using the given [model] and [kGain], with feed forward.
 *
 * Inputs:
 * 1. Either: reference vector or [MotionState] of the reference vector
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
) : Combine<Any, Vec, Vec>() {

    init {
        require(kGain.matches(model.stateStructure, model.inputStructure))
    }

    //flatten model
    private val kGain = kGain.toImmutableMat()
    private val kFF = plantInversionKFF(model, feedForwardQRCost)

    override fun combine(a: Any, b: Vec): Vec {
        //we don't care about elapsed seconds.
        val (r, r1) = getRefs(a)
        val x = b
        return kGain(r - x) + kFF(r1 - model.A * r)
    }

    @Suppress("UNCHECKED_CAST")
    private fun getRefs(r: Any): Pair<Vec, Vec> = when (r) {
        is MotionState<*> -> {
            r as MotionState<Vec>
            r.s to r.s + r.v * model.period + r.a * (model.period.squared() / 2)
        }
        is Vec -> r to r
        else -> throw ClassCastException()
    }

    /** Reference [BlocksConfig.Input] */
    val reference: BlocksConfig.Input<Any> get() = first
    /** State [BlocksConfig.Input] */
    val state: BlocksConfig.Input<Vec> get() = second
}