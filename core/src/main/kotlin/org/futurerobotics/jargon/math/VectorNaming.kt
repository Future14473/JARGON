package org.futurerobotics.jargon.math

import koma.extensions.get
import org.futurerobotics.jargon.util.zipForEachIndexed

/**
 * A way to represent the values in a vector with value names and units, when it makes sense.
 *
 * Each value in the vector has a corresponding name and unit, given in [names] and [units].
 *
 * When [toString] is called, it return a string of the given vector using these names.
 *
 * This makes debugging or presenting vector related info easier.
 */
class VectorNaming {

    private val names: List<String>
    private val units: List<String>

    constructor(names: List<String>, units: List<String>) {
        require(names.size == units.size) { "names and units lists must be same size" }
        this.names = names.toList()
        this.units = units.toList()
    }

    constructor(namesAndUnits: List<Pair<String, String>>) {
        namesAndUnits.spliterator()
        names = namesAndUnits.map { it.first }
        units = namesAndUnits.map { it.second }
    }

    /**
     * Creates a empty naming with the given size.
     */
    constructor(size: Int) {
        names = List(size) { "" }
        units = List(size) { "" }
    }

    /**
     * The number of dimensions this vector naming is for.
     */
    val size: Int get() = names.size

    /**
     * Gets a string representation of the supplied [vector] with this vector naming.
     */
    fun toString(vector: Mat): String {
        require(vector.isVector()) { "Supplied matrix must be a vector" }
        require(vector.numElements() == size) { "Supplied vector must have same size as this naming." }
        return internalToString(vector)
    }

    /**
     * Returns if the given [matrix] is a vector and is valid for this naming
     */
    infix fun matchedBy(matrix: Mat): Boolean = matrix.isVector() && matrix.numElements() == size

    private fun internalToString(vector: Mat): String = buildString {
        appendln('[')
        names.zipForEachIndexed(units) { index, name, unit ->
            append(name)
            append(':')
            append(' ')
            append(vector[index])
            append(' ')
            appendln(unit)
        }
        appendln(']')
    }
}

/** Returns if this matrix is a vector that is valid for the given [naming] */
infix fun Mat.matches(naming: VectorNaming): Boolean = naming matchedBy this

/** Returns if this matrix is valid for naming by row and column, using [rowNaming] and  [colNaming] */
fun Mat.matches(rowNaming: VectorNaming, colNaming: VectorNaming): Boolean =
    numRows() == rowNaming.size && numCols() == colNaming.size