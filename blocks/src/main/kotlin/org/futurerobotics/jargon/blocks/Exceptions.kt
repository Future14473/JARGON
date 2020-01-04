package org.futurerobotics.jargon.blocks

/** An exception thrown when an illegal block configuration is detected. */
class IllegalBlockConfigurationException : RuntimeException {

    constructor() : super()
    constructor(message: String) : super(message)
    constructor(message: String, cause: Throwable) : super(message, cause)
    constructor(cause: Throwable) : super(cause)
}

/** Thrown when an exception is made when running a given `block`. */
class BlockProcessException : RuntimeException {

    constructor(message: String, cause: Throwable) : super(message, cause)
    constructor(message: String) : super(message)
    constructor(cause: Throwable) : super(cause)
    constructor() : super()
}
