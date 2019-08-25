package org.futurerobotics.temporaryname.control.plant

/**
 * Represents a Plant; i.e. the system of actuators and measurement of a control system
 * -- your motors and sensors; or the model representing such.
 *
 * This does not _have to_ include linear algebra and fancy math.
 *
 * This is also the highest level direct interface to the hardware.
 *
 * @param Signal the class to represent the Signal type; for example a vector of motor voltages
 * @param Measurement the class to represent the Measurement;  for example motor speed/position + gyro
 */
interface Plant<in Signal,out Measurement> {
    /**
     * Sets the signal on the actuators, and returns a measurement.
     *
     * (using zero-order hold -- signal values held constant).
     */
    fun signalAndMeasure(signal: Signal): Measurement

    /**
     * Safely stops all actuators on interrupted shutdown.
     */
    fun stop()
}