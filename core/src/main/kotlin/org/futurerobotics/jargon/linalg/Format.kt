@file:JvmName("MatFormat")

package org.futurerobotics.jargon.linalg

import java.text.DecimalFormat

/**
 * Formats this matrix as a matrix literal.
 */
fun Mat.formatLiteral(): String = buildString {
    appendln("mat[")
    repeat(rows) { r ->
        repeat(cols - 1) { c ->
            append(this@formatLiteral[r, c].format())
            append(',')
        }
        append(this@formatLiteral[r, cols - 1].format())
        appendln(" end")
    }
    appendln(']')
}

/**
 * Formats this matrix in a human readable format
 */
fun Mat.formatReadable(): String = buildString {
    repeat(rows) { r ->
        append("| ")
        repeat(cols - 1) { c ->
            append(this@formatReadable[r, c].format())
            append(' ')
        }
        append(this@formatReadable[r, cols - 1].format())
        appendln(" |")
    }
}

private val decimalFormat = DecimalFormat(" 0.0000;-#")
private fun Double.format(): String = "%-8s".format(decimalFormat.format(this))
