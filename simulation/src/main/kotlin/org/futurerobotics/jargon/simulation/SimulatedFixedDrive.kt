package org.futurerobotics.jargon.simulation

import org.futurerobotics.jargon.control.AbstractBlock
import org.futurerobotics.jargon.control.Block.InOutOrder.IN_FIRST
import org.futurerobotics.jargon.control.Block.InOutOrder.OUT_FIRST
import org.futurerobotics.jargon.control.Block.Processing.ALWAYS
import org.futurerobotics.jargon.control.castToDoubleList
import org.futurerobotics.jargon.control.castToVec
import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.math.Pose2d
import org.futurerobotics.jargon.mechanics.FixedDriveModel
import org.futurerobotics.jargon.statespace.LinearDriveModels
import java.util.*
import kotlin.math.roundToInt

/**
 * Represents a simulated drive interface.
 *
 * Right now only [FixedDriveModel]s are supported.
 */
interface SimulatedDrive {
    /**
     * Gets the current positions of the wheels.
     */
    val currentPositions: Vec
    /**
     * Gets the current velocities of the wheels.
     */
    val currentVelocities: Vec

    /**
     * Gets the current pose of the bot.
     */
    val currentPose: Pose2d

    /**
     * Updates the simulation, using the given [volts] vector and the loop [time] elapsed.
     */
    fun update(volts: Vec, time: Double)
}

/**
 * Base class for a simulated drive.
 *
 * @param driveModel the model to use
 * @param voltageNoise the voltage signal noise covariance
 * @param measurementNoise the measurement noise covariance
 * @param timeStep the minimum amount of time passed between samples. Because I don't like euler approximations.
 */
class SimulatedFixedDrive(
    private val driveModel: FixedDriveModel,
    private val voltageNoise: Mat,
    private val measurementNoise: Mat,
    private val timeStep: Double = 0.01,
    private val random: Random = Random()
) : SimulatedDrive {
    constructor(
        model: FixedDriveModel,
        perturb: Perturber<FixedDriveModel>,
        random: Random,
        voltageNoise: Mat,
        measurementNoise: Mat,
        timeStep: Double = 0.01
    ) : this(perturb.perturb(model, random), voltageNoise, measurementNoise, timeStep, random)

    init {
        require(voltageNoise.isSquare) { "Voltage noise $voltageNoise must be square" }
        require(measurementNoise.isSquare) { "Measurement noise $measurementNoise must be square" }
        require(timeStep > 0) { "Time step $timeStep must be > 0" }
    }

    override val currentPositions: Vec = zeroVec(driveModel.numWheels)
    override var currentVelocities: Vec = zeroVec(driveModel.numWheels) //== state
    override var currentPose: Pose2d = Pose2d.ZERO
    //wheel velocity controller
    private val wheelSSModel = LinearDriveModels.wheelVelocityController(driveModel).discretize(timeStep)

    override fun update(volts: Vec, time: Double) {
        repeat((time / timeStep).roundToInt()) {
            singleStep(volts)
        }
    }

    private fun singleStep(volts: Vec) {
        val realVolts = volts + voltageNoise * normRandVec(voltageNoise.rows)
        val pastVelocities = currentVelocities
        currentVelocities = wheelSSModel.processState(pastVelocities, realVolts)
        val diff = (pastVelocities + currentVelocities) * (timeStep / 2)
        currentPositions += diff
        currentPose += driveModel.getEstimatedVelocity(diff.asList())
    }
}

/**
 * Simulated drive input block.
 *
 * Inputs:
 * 1. Voltage inputs; either a [Vec], List<Double> or DoubleArray/double[]
 * 2. Elapsed time.
 *
 * Outputs:
 * 1. A List<Double> of motor positions in radians
 * 2. A List<Double> of motor velocities in radians
 */
class SimulatedDriveInput(private val drive: SimulatedFixedDrive) : AbstractBlock(2, 1, OUT_FIRST, ALWAYS) {
    private fun writeOutputs(outputs: MutableList<Any?>) {
        outputs[0] = drive.currentPositions.castToDoubleList()
        outputs[1] = drive.currentVelocities.castToDoubleList()
    }

    override fun init(outputs: MutableList<Any?>) {
        writeOutputs(outputs)
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        drive.update(outputs[0]!!.castToVec(), outputs[1] as Double)
        writeOutputs(outputs)
    }
}

/**
 * Simulated gyroscope measurement.
 */
class SimulatedGyro(
    private val drive: SimulatedFixedDrive,
    private val noiseStd: Double,
    private val random: Random = Random()
) : AbstractBlock(
    0, 1, IN_FIRST, ALWAYS
) {
    override fun init(outputs: MutableList<Any?>) {
        outputs[0] = drive.currentPose.heading + random.nextGaussian() * noiseStd
    }

    override fun process(inputs: List<Any?>, outputs: MutableList<Any?>) {
        outputs[0] = drive.currentPose.heading + random.nextGaussian() * noiseStd
    }

}