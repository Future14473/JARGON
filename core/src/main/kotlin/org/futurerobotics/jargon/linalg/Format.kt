package org.futurerobotics.jargon.linalg

import java.text.DecimalFormat

/**
 * Formats this matrix as a matrix literal.
 */
fun Mat.formatLiteral(): String = buildString {
    appendln("mat[")
    repeat(rows) { i ->
        repeat(cols - 1) { j ->
            append(this@formatLiteral[i, j].format())
            append(',')
        }
        append(this@formatLiteral[i, cols - 1].format())
        appendln(" end")
    }
    appendln(']')
}

/**
 * Formats this matrix in a human readable format
 */
fun Mat.formatReadable(): String = buildString {
    repeat(rows) { i ->
        append("| ")
        repeat(cols - 1) { j ->
            append(this@formatReadable[i, j].format())
            append(' ')
        }
        append(this@formatReadable[i, cols - 1].format())
        appendln(" |")
    }
}

private val decimalFormat = DecimalFormat(" 0.0000;-#")
private fun Double.format(): String = "%-8s".format(decimalFormat.format(this))