package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.control.CombineBlock
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.matches
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.MotionState2
import org.futurerobotics.jargon.mechanics.MotionState3


/**
 * A state space controller using the given [model] and [kGain], with feed forward.
 *
 * Inputs:
 * 1. Either: reference vector, or [MotionState2]/[MotionState3] of the reference vector
 * 2. Current state vector
 *
 * Outputs:
 * 3. Signal vector
 *
 * @param model the [DiscreteLinSSModel] to use
 * @param kGain the kGain matrix
 * @param feedForwardQRCost the [QRCost] to use in calculation feed-forward gain; null if only pseudo-inverse is to be used.
 */
class SSControllerWithFF(
    private val model: DiscreteLinSSModel,
    kGain: Mat,
    feedForwardQRCost: QRCost? = null
) : CombineBlock<Any, Vec, Vec>() {

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
        is MotionState3<*> -> {
            r as MotionState3<Vec>
            r.s to r.s + r.v * model.period + r.a * (model.period.squared() / 2)
        }
        is MotionState2<*> -> {
            r as MotionState2<Vec>
            r.s to r.s + r.v * model.period
        }
        is Vec -> r to r
        else -> throw ClassCastException()
    }

}