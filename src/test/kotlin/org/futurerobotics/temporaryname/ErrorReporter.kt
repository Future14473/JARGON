@file:Suppress("MemberVisibilityCanBePrivate")

package org.futurerobotics.temporaryname

import kotlin.math.abs

class ErrorReporter {
    var maxError: Double = Double.NEGATIVE_INFINITY
        private set
    var maxErrorMessage: String? = null
        private set
    var totalError: Double = 0.0
        private set
    var errorPointCount: Int = 0
        private set
    val averageError: Double
        get() = totalError / errorPointCount

    inline fun addError(error: Double, message: () -> String? = { null }) {
        val err = abs(error)
        val maybeMessage: String? = if (err > maxError) message() else null
        addError(error, maybeMessage)
    }

    fun addError(error: Double, message: String?) {
        val err = abs(error)
        if (err > maxError) {
            maxError = err
            maxErrorMessage = message
        }
        totalError += error
        errorPointCount++
    }

    fun report(): String {
        return if (errorPointCount == 0) "No errors recorded" else """
            Max Error: ${"%6f".format(maxError)}: ${maxErrorMessage ?: ""}
            Total Error: ${"%6f".format(totalError)}
            Average Error: ${"%6f".format(averageError)}
            Errors recorded: $errorPointCount
            """.trimIndent()
    }
}

fun reportError(block: ErrorReporter.() -> Unit): ErrorReporter = ErrorReporter().apply(block)
