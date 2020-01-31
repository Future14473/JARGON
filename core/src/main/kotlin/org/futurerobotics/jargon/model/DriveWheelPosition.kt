package org.futurerobotics.jargon.model

import org.futurerobotics.jargon.math.Vector2d
import org.futurerobotics.jargon.math.epsEq
import org.futurerobotics.jargon.model.DriveWheelTransmission.Companion.fromPosition

/**
 * Represents the position and orientation of a wheel that is fixed in place (not a swerve wheel or similar)
 *
 * @see DriveWheelTransmission
 * @see PoweredDriveWheelModel
 */
open class DriveWheelPosition
protected constructor(
    /** Location of this wheel relative to the center. */
    val location: Vector2d,
    /**
     * A unit vector in the direction this wheel is facing, such that a positive velocity means
     * it is pushing in this direction, or a positive output torque results in a force in this direction.
     */
    val orientation: Vector2d
) {

    init {
        require(location.isFinite()) { "location ($location) must be finite" }
        require(orientation.length epsEq 1.0) { "orientation ($orientation) must be a unit vector" }
    }

    /**
     * The ratio of the wheel's tangential velocity with the bot's angular velocity (if the
     * wheel does not slip).
     */
    val wheelVelPerBotAngularVel: Double get() = location cross orientation

    companion object {
        /**
         * Creates a [DriveWheelPosition].
         *
         * @param location the position the wheel is located relative to the center.
         * @param angle the angle this wheel is facing in radians such that a positive wheel velocity means
         *  the drive is moving in this direction
         */
        @JvmStatic
        fun of(
            location: Vector2d,
            angle: Double
        ): DriveWheelPosition {
            require(angle.isFinite()) { "angle ($angle) must be finite." }
            return DriveWheelPosition(
                location,
                Vector2d.polar(1.0, angle)
            )
        }
    }
}

/**
 * A [DriveWheelPosition] that additionally includes information about it's wheel radius,
 * and possible gearing from input to output, so it can be used for measurements.
 *
 * This can be created using [fromPosition] factory function.
 */
abstract class DriveWheelTransmission
internal constructor(
    location: Vector2d,
    orientation: Vector2d
) : DriveWheelPosition(location, orientation), Gearing {

    /** The ratio of the inputted velocity with the bot's angular velocity, assuming the wheel does not slip. */
    val inputVelPerBotAngularVel: Double get() = inputVelPerOutputVel * wheelVelPerBotAngularVel

    companion object {
        /**
         * Creates a [DriveWheelTransmission] from a another [DriveWheelPosition].
         *
         * @param location the position the wheel is located relative to the center.
         * @param angle the angle this wheel is facing in radians such that a positive wheel velocity means
         *  the drive is moving in this direction
         * @param wheelRadius the radius of the wheel.
         * @param gearRatio the output:input gear ratio. Higher gear ratio means more torque, less speed at output.
         */
        @JvmStatic
        fun of(
            location: Vector2d,
            angle: Double,
            wheelRadius: Double,
            gearRatio: Double
        ): DriveWheelTransmission =
            fromPosition(of(location, angle), wheelRadius, gearRatio)

        /**
         * Creates a [DriveWheelTransmission] from a another [DriveWheelPosition].
         *
         * @param wheelModel the [DriveWheelPosition]
         * @param wheelRadius the radius of the wheel
         * @param gearRatio the output:input gear ratio. Higher gear ratio means more torque, less speed at output.
         */
        @JvmStatic
        fun fromPosition(
            wheelModel: DriveWheelPosition,
            wheelRadius: Double,
            gearRatio: Double
        ): DriveWheelTransmission {
            val gearRatioOverWheelRadius = checkWG(wheelRadius, gearRatio)
            return object : DriveWheelTransmission(
                wheelModel.location,
                wheelModel.orientation
            ) {
                override val outputVelPerInputVel: Double get() = 1 / inputVelPerOutputVel
                override val inputVelPerOutputVel: Double = gearRatioOverWheelRadius
            }
        }
    }
}

/**
 * A [DriveWheelPosition] combined with a [TorquePowerModel] so that now the wheel is powered.
 *
 * This is a subclass of [DriveWheelTransmission] and [PoweredDriveWheelModel],
 * and implements [ForcePowerModel].
 */
class PoweredDriveWheelModel
private constructor(
    location: Vector2d,
    orientation: Vector2d,
    poweredWheelModel: PoweredWheelModel
) : DriveWheelTransmission(location, orientation),
    ForcePowerModel by poweredWheelModel {

    companion object {
        /**
         * Creates a [PoweredDriveWheelModel] from a [DriveWheelPosition], gear ratio information, and
         * a [TorquePowerModel].
         *
         * @param power The already geared power model.
         */
        @JvmStatic
        fun fromPosition(
            position: DriveWheelPosition,
            wheelRadius: Double,
            gearRatio: Double,
            power: TorquePowerModel
        ): PoweredDriveWheelModel {
            return PoweredDriveWheelModel(
                position.location,
                position.orientation,
                PoweredWheelModel.of(wheelRadius, gearRatio, power)
            )
        }

        /**
         * Creates a [PoweredDriveWheelModel] from a [DriveWheelPosition] and
         * a [TorquePowerModel]
         * (which can be either directly a [DcMotorModel], or a [ModifiedPowerModel] for more options).
         *
         * @param power The already geared power model.
         */
        @JvmStatic
        fun fromTransmission(
            wheel: DriveWheelTransmission,
            power: TorquePowerModel
        ): PoweredDriveWheelModel = PoweredDriveWheelModel(
            wheel.location,
            wheel.orientation,
            PoweredWheelModel.from(wheel.inputVelPerOutputVel, power)
        )

        /**
         * Creates a [PoweredDriveWheelModel] from a [DriveWheelPosition], gear ratio information, and
         * a [TorquePowerModel].
         *
         * @param power The already geared power model.
         */
        @JvmStatic
        fun fromPoweredWheel(
            position: DriveWheelPosition,
            power: PoweredWheelModel
        ): PoweredDriveWheelModel =
            PoweredDriveWheelModel(
                position.location,
                position.orientation,
                power
            )
    }
}
