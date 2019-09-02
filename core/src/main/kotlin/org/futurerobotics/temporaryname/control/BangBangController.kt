package org.futurerobotics.temporaryname.control

/**
 * A controller that outputs [lessThanOutput] as a signal when the state is less than the reference, [greaterThanOutput] when
 * greater than the reference, and [zeroOutput] if the reference equals the output, and that's it.
 *
 * One of the most basic controllers one can think of.
 */
/*
TODO
Perhaps just ditch this, nonessential
 */
open class BangBangController<Signal : Any>(
    private val lessThanOutput: Signal,
    private val greaterThanOutput: Signal,
    private val zeroOutput: Signal
) : Controller<Double, Double, Signal> {

    override lateinit var signal: Signal
    override fun update(reference: Double, currentState: Double, elapsedSeconds: Double) {
        val comp = currentState.compareTo(reference)
        signal = when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> zeroOutput
        }
    }

    override fun start() {}
    override fun stop() {}
}