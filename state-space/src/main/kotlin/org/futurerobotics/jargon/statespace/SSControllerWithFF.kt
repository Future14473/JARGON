package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.control.BaseController
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.matches
import org.futurerobotics.jargon.math.squared
import org.futurerobotics.jargon.mechanics.MotionState2
import org.futurerobotics.jargon.mechanics.MotionState3


/**
 * Common components of [SSControllerWithFF] and [SSControllerWithEFF]
 *
 * @param model the [DiscreteSSModel] to use
 * @param kGain the kGain matrix
 * @param feedForwardQRCost the [QRCost] to use in calculation feed-forward gain; null if only pseudo-inverse is to be used.
 */
abstract class BaseSSControllerWithFF<T : MotionState2<Vec>>(
    protected val model: DiscreteSSModel,
    kGain: Mat,
    feedForwardQRCost: QRCost? = null
) : BaseController<T, Vec, Vec>() {

    init {
        require(kGain.matches(model.stateStructure, model.inputStructure))
    }

    //flatten model
    private val kGain: Mat = kGain.copy()
    private val kFF = plantInversionKFF(model, feedForwardQRCost)
    override fun getSignal(reference: T, currentState: Vec, elapsedSeconds: Double): Vec {
        //we don't care about elapsed seconds.
        val x = currentState
        val r = reference.s
        val r1 = getNextRef(reference) //is this good enough? Does this translate for discrete things?
        return kGain(r - x) + kFF(r1 - model.A * r)
    }

    /**
     * Gets the required next after a period of T (discrete) given the current reference motion (continuous)
     */
    protected abstract fun getNextRef(
        reference: T
    ): Vec

    override fun stop() {
    }
}

/**
 * Represents a state-space controller with plant-inversion feed-forward.
 *
 * @param model the model to use
 * @param kGain the inputs x states gain matrix
 * @param feedForwardQRCost the cost used to calculate plant inversion; if null, no cost matrices are used.
 * @see SSControllerWithEFF
 */
@Suppress("PrivatePropertyName")
class SSControllerWithFF @JvmOverloads constructor(
    model: DiscreteSSModel,
    kGain: Mat,
    feedForwardQRCost: QRCost? = null
) : BaseSSControllerWithFF<MotionState2<Vec>>(model, kGain, feedForwardQRCost) {

    override fun getNextRef(
        reference: MotionState2<Vec>
    ) = reference.s + reference.v * model.period

}

/**
 * Represents a state-space controller with an plant-inversion feed-forward _that also uses acceleration
 * in [MotionState3] for slightly better feed-forward._
 *
 * @param model the model to use
 * @param kGain the inputs x states gain matrix
 * @param feedForwardQRCost the cost used to calculate plant inversion; if null, no cost matrices are used.
 * @see SSControllerWithFF
 */
@Suppress("PrivatePropertyName")
class SSControllerWithEFF @JvmOverloads constructor(
    model: DiscreteSSModel,
    kGain: Mat,
    feedForwardQRCost: QRCost? = null
) : BaseSSControllerWithFF<MotionState3<Vec>>(model, kGain, feedForwardQRCost) {
    override fun getNextRef(reference: MotionState3<Vec>): Vec {
        val t = model.period
        val (s, v, a) = reference
        return s + v * t + a * (t.squared() / 2)
    }
}