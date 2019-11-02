package org.futurerobotics.jargon.learning

/**
 * Represents a generic predictor, which estimates some output based on inputs. This is allowed
 * to be mutable.
 */
interface Predictor<Input, Output> {

    /** Predicts output based on given [input] */
    fun predict(input: Input): Output

    /** Predict outputs based on given [inputs] */
    @JvmDefault
    fun predict(inputs: List<Input>): List<Output> = inputs.map { predict(it) }
}

/** Common interface of all fitters. */
interface AnyFitter<Input, Output, Pred : Predictor<Input, Output>> {

    /** Gets a value representing the cost of this fitter based on the given [inputs] and [outputs]. */
    fun cost(predictor: Pred, inputs: List<Input>, outputs: List<Output>): Double
}

/**
 * Fits a [Pred]ictor with actual data.
 */
interface Fitter<Input, Output, Pred : Predictor<Input, Output>> :
    AnyFitter<Input, Output, Pred> {

    /** Fit this predictor by considering all the given [inputs] and [outputs] data, but need not until convergence. */
    fun fitOnce(predictor: Pred, inputs: List<Input>, outputs: List<Output>)
}

/**
 * Fits a [Pred]ictor but also can also operate stocahstically.
 */
interface StochasticFitter<Input, Output, Pred : Predictor<Input, Output>> :
    AnyFitter<Input, Output, Pred> {

    /** Runs a single stochastic update using the given [input] and [output] data */
    fun stochasticUpdate(predictor: Pred, input: Input, output: Output)
}
