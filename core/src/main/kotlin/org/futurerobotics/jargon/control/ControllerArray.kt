package org.futurerobotics.jargon.control

import org.futurerobotics.jargon.linalg.*

//TODO: test
/**
 * A controller that operates on vectors, by having an array of controllers each operating on one vector
 * component. For example, this can be used to have several controllers for each wheel on a drive
 * train, while being able to be used and interpreted as one controller.
 *
 * @param size the size of the vector expected
 * @param provideController a function that creates each controller.
 */
class ControllerArray(
    private val size: Int,
    provideController: () -> SimpleController<Double, Double, Double>
) : SimpleController<Vec, Vec, Vec> {

    private val controllers = Array(size) { provideController() }

    override var signal: Vec = Vec(size)
        private set

    override fun update(reference: Vec, currentState: Vec, elapsedTimeInNanos: Long): Vec {
        return Vec(size) { i ->
            controllers[i].update(reference[i], currentState[i], elapsedTimeInNanos)
        }
    }
}
