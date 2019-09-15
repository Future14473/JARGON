@file:Suppress("MemberVisibilityCanBePrivate")

package org.futurerobotics.temporaryname

import kotlin.math.abs

/**
 * Helps in reporting error metrics.
 */
class ErrorReporter {
    /**
     * The maximum error reported so far.
     */
    var maxError: Double = Double.NEGATIVE_INFINITY
        private set
    /**
     * The message at the maximum error, if any.
     */
    var maxErrorMessage: String? = null
        private set
    /**
     * The sum of all the errors reported so far.
     */
    var totalError: Double = 0.0
        private set
    /**
     * The number of errors recorded.
     */
    var errorPointCount: Int = 0
        private set
    /**
     * The average error.
     */
    val averageError: Double
        get() = totalError / errorPointCount

    /**
     * Records an error, and possibly records [lazyMessage] at that point if it is the maximum error so far.
     */
    inline fun addError(error: Double, lazyMessage: () -> String? = { null }) {
        val err = abs(error)
        val maybeMessage: String? = if (err > maxError) lazyMessage() else null
        addError(error, maybeMessage)
    }

    /**
     * Records an error, and possibly records [message] at that point if it is the maximum error so far.
     */
    fun addError(error: Double, message: String? = null) {
        val err = abs(error)
        if (err > maxError) {
            maxError = err
            maxErrorMessage = message
        }
        totalError += error
        errorPointCount++
    }

    /**
     * Generates an String representing a error report.
     */
    fun report(): String = if (errorPointCount == 0) "No errors recorded" else """
        Max Error: ${"%6f".format(maxError)}: ${maxErrorMessage ?: ""}
        Total Error: ${"%6f".format(totalError)}
        Average Error: ${"%6f".format(averageError)}
        Errors recorded: $errorPointCount
        """.trimIndent()
}

/**
 * DSL for using [ErrorReporter]
 */
fun reportError(block: ErrorReporter.() -> Unit): ErrorReporter = ErrorReporter().apply(block)
