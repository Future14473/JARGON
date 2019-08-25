package org.futurerobotics.temporaryname.control.controller

/**
 * A controller that outputs [lessThanOutput] as a signal when the state is less than the reference, [greaterThanOutput] when
 * greater than the reference, and [zeroOutput] if the reference equals the output, and that's it.
 *
 * One of the most basic controllers one can think of.
 */
open class BangBangController<Signal>(
    private val lessThanOutput: Signal,
    private val greaterThanOutput: Signal,
    private val zeroOutput: Signal
) : ControllerIgnoreDeriv<Double, Signal>() {
    override fun process(current: Double, reference: Double, elapsedNanos: Long): Signal {
        val comp = current.compareTo(reference)
        return when {
            comp < 0 -> lessThanOutput
            comp > 0 -> greaterThanOutput
            else -> zeroOutput
        }
    }

    override fun reset() {}
}