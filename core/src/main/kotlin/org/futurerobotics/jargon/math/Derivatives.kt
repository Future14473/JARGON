package org.futurerobotics.jargon.math

///**
// * Represents information about value, first, and second derivatives of a numerical value.
// */
//interface Derivatives<T> {
//
//    /** The value */
//    val value: T
//    /** The value's derivative */
//    val vel: T
//    /** The value's second derivative */
//    val accel: T
//
//    /** [value] */
//    @JvmDefault
//    operator fun component1(): T = value
//
//    /** [vel] */
//    @JvmDefault
//    operator fun component2(): T = vel
//
//    /** [accel] */
//    @JvmDefault
//    operator fun component3(): T = accel
//}
//
///** A simple [Derivatives] implementation that holds values in fields */
//class ValueDerivatives<T>(override val value: T, override val vel: T, override val accel: T) :
//    Derivatives<T>, java.io.Serializable {
//
//    companion object {
//        private const val serialVersionUID = -7402403796313681521
//    }
//}
//
