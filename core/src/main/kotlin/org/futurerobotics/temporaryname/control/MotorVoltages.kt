package org.futurerobotics.temporaryname.control

/**
 * A list of voltages, usually used as a drive signal.
 */
class MotorVoltages internal constructor(private val voltages: List<Double>) : List<Double> by voltages {

    companion object {
        /**
         * Constructs a [MotorVoltages] based on the supplied [voltages]
         */
        @JvmStatic
        fun of(voltages: List<Double>): MotorVoltages = MotorVoltages(voltages.toList())

        /**
         * Constructs a [MotorVoltages] based on the supplied [voltages]
         */
        @JvmStatic
        fun of(vararg voltages: Double): MotorVoltages = MotorVoltages(voltages.toList())
    }
}

/**
 * A list of positions, usually used as a measurement.
 */
class MotorPositions internal constructor(private val positions: List<Double>) : List<Double> by positions {

    companion object {
        /**
         * Constructs a [MotorPositions] based on the supplied [positions]
         */
        @JvmStatic
        fun of(positions: List<Double>): MotorPositions = MotorPositions(positions.toList())

        /**
         * Constructs a [MotorPositions] based on the supplied [positions]
         */
        @JvmStatic
        fun of(vararg positions: Double): MotorPositions = MotorPositions(positions.toList())
    }
}

