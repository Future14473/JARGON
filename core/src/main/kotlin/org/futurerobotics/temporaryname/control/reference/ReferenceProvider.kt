package org.futurerobotics.temporaryname.control.reference

import org.futurerobotics.temporaryname.system.StartStoppable

/**
 * A generic reference provider that takes in [InState] and produces reference states [OutState]
 */
interface GenericReferenceProvider<in InState, out OutState> : StartStoppable {

    /**
     * Given the [currentState] ([InState]) and elapsed time in nanoseconds ([elapsedNanos]),
     *
     * returns an reference [OutState]
     *
     * [elapsedNanos] will be -1 on first iteration of the control loop.
     */
    fun update(currentState: InState, elapsedNanos: Long): OutState
}

/**
 * A generic reference provider that takes in the current [State] to update and returns a reference [State].
 */
interface ReferenceProvider<State> : GenericReferenceProvider<State, State> {
    /**
     * Given the [currentState] and elapsed time in nanoseconds ([elapsedNanos]),
     * returns a reference [State]
     *
     * [elapsedNanos] will be -1 on first iteration of the control loop.
     */
    override fun update(currentState: State, elapsedNanos: Long): State
}

/**
 * A reference provider that takes in [State] and returns [State] and [StateDeriv], usually used for feed-forwards.
 */
interface DerivRefProvider<State : Any, StateDeriv> :
    GenericReferenceProvider<State, Pair<State, StateDeriv?>> {

    /**
     * Given the [currentState] and elapsed time in nanoseconds ([elapsedNanos]),
     * returns:
     * - a reference [State]
     * - a reference [StateDeriv], if known, else `null`.
     *
     * [elapsedNanos] will be -1 on first iteration of the control loop.
     */
    override fun update(currentState: State, elapsedNanos: Long): Pair<State, StateDeriv?>
}

/**
 * Returns this [GenericReferenceProvider] as a [DerivRefProvider] that returns a ]
 */
fun <S : Any, SD> GenericReferenceProvider<S, S>.asDerivativeReferenceProvider(): DerivRefProvider<S, SD> =
    object : DerivRefProvider<S, SD>, StartStoppable by this {
        override fun update(currentState: S, elapsedNanos: Long): Pair<S, SD?> {
            return this@asDerivativeReferenceProvider.update(currentState, elapsedNanos) to null
        }
    }

/**
 * Provides the reference [State] to be fed into a controller, also including expected first derivative info,
 * and additional info (which may be state's SecondDeriv, or other.
 * ([StateDeriv] and [StateSecondDeriv]) possibly to be used for feed-forwards for extended controllers.
 *
 * This does not _have to_ include linear algebra and fancy math.
 *
 * Nothing is stopping you from "chaining" [GenericReferenceProvider] together to create a "reference pipeline", so to speak.
 */
interface ExtendedDerivRefProvider<State, out StateDeriv : Any, out StateSecondDeriv : Any>
    : GenericReferenceProvider<State, Triple<State, StateDeriv?, StateSecondDeriv?>> {
    /**
     * Given the [currentState] and elapsed time in nanoseconds ([elapsedNanos]),
     * returns:
     * - a reference/target [State],
     * - the expected reference [StateDeriv], if available/known, else null,
     * - the expected reference [StateSecondDeriv], if available/known, else null.
     *
     * [elapsedNanos] will be -1 on first iteration of the control loop.
     */
    override fun update(currentState: State, elapsedNanos: Long): Triple<State, StateDeriv?, StateSecondDeriv?>
}
