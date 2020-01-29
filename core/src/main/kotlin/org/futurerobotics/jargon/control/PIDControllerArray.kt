package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*

/**
 * A controller that operates on vectors by having a separate PID controller for
 * each vector component. This can be used, for example, to easily configure a PID controller on multiple arrays.
 */
class PIDControllerArray(
    private val size: Int,
    coefficients: PIDCoefficients
) : SimpleController<Vec, Vec, Vec> {

    private val controllers = Array(size) { PIDController(coefficients.toExtendedCoefficients()) }

    override var signal: Vec = Vec(size)
        private set

    override fun reset() {
        controllers.forEach { it.reset() }
    }

    override fun update(reference: Vec, currentState: Vec, elapsedTimeInNanos: Long): Vec {
        val result = Vec(size)
        repeat(size) { i ->
            result[i] = controllers[i].update(reference[i], currentState[i], elapsedTimeInNanos)
        }
        return result
    }
}
