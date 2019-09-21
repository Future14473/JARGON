package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.util.forEachZippedIndexed
import org.futurerobotics.jargon.util.repeatedList

/**
 * A way to represent the values in a vector with value names and units, when it makes sense.
 *
 * Each value in the vector has a corresponding name and unit, given in [names] and [units].
 *
 * When [toString] is called, it return a string of the given vector using these names.
 *
 * This makes debugging or presenting vector related info easier.
 */
class VectorStructure {

    private val names: List<String>
    private val units: List<String>

    constructor(names: List<String>, units: List<String>) {
        require(names.size == units.size) { "names and units lists must be same size" }
        this.names = names.toList()
        this.units = units.toList()
    }
    constructor(names: List<String>) {
        this.names = names.toList()
        this.units = repeatedList(names.size,"")
    }

    /**
     * Creates a empty naming with the given size.
     */
    constructor(size: Int) {
        val list = repeatedList(size, "")
        names = list
        units = list
    }

    /**
     * The number of dimensions this vector naming is for.
     */
    val size: Int get() = names.size

    /**
     * Gets a string representation of the supplied [vector] with this vector naming.
     */
    fun toString(vector: Vec): String {
        require(this matchedBy vector) { "Supplied vector must have same size as this naming." }
        return internalToString(vector)
    }

    /**
     * Returns if the given [vector] is a vector and is valid for this naming
     */
    infix fun matchedBy(vector: Vec): Boolean = vector.dimension == size

    private fun internalToString(vector: Vec): String = buildString {
        appendln('[')
        forEachZippedIndexed(names, units) { index, name, unit ->
            append(name)
            append(':')
            append(' ')
            append(vector[index])
            append(' ')
            appendln(unit)
        }
        append(']')
    }
}

/** Returns if this matrix is a vector that is valid for the given [structure] */
infix fun Vec.matches(structure: VectorStructure): Boolean = structure matchedBy this

/** Returns if this matrix is valid for naming by row and column, using [rowStructure] and  [colStructure] */
fun Mat.matches(rowStructure: VectorStructure, colStructure: VectorStructure): Boolean =
    rows == rowStructure.size && cols == colStructure.size