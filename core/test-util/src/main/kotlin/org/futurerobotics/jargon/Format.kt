package org.futurerobotics.jargon

/** Normalizes newlines. */
fun String.normalize(): String =
    replace("\r\n", "\n")
        .replace("\r", "\n")