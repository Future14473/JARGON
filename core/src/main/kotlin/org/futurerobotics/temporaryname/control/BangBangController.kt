package org.futurerobotics.temporaryname.control

/**
 * A controller that outputs [lessThanOutput] as a signal when the state is less than the reference, [greaterThanOutput]
 * when greater than the reference, and [zeroOutput] if the reference equals the output, and that's it.
 *
 * One of the more basic controllers.
 */
 class BangBangController<State: Comparable<State>, Signal : Any>(
    private val lessThanOutput: Signal,
    private val greaterThanOutput: Signal,
    private val zeroOutput: Signal
) : Controller<State, State, Signal> {

    override lateinit var signal: Signal
    override fun update(reference: State, currentState: State, elapsedSeconds: Double) {
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