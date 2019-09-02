package org.futurerobotics.temporaryname.mechanics

/**
 * Base class for a named list of values.
 */
abstract class NamedDoubleList protected constructor(private val values: List<Double>) :
    List<Double> by values

/**
 * A list of motor voltages, usually used as a drive signal.
 */
class MotorVoltages : NamedDoubleList {

    internal constructor(values: List<Double>, dummy: Boolean) : super(values)
    constructor(values: List<Double>) : super(values.toList())
    constructor(vararg values: Double) : super(values.toList())
}

/**
 * A list of motor positions, usually used as a measurement.
 */
class MotorPositions : NamedDoubleList {

    internal constructor(values: List<Double>, dummy: Boolean) : super(values)
    constructor(values: List<Double>) : super(values.toList())
    constructor(vararg values: Double) : super(values.toList())
}

/**
 * A list of motor velocities, usually used as a reference.
 */
class MotorVelocities : NamedDoubleList {

    internal constructor(values: List<Double>, dummy: Boolean) : super(values)
    constructor(values: List<Double>) : super(values.toList())
    constructor(vararg values: Double) : super(values.toList())
}
