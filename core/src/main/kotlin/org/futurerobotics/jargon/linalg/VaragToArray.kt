package org.futurerobotics.jargon.linalg

/**
 * Converts vararg elements in the style of (a,b to c,d) to a 2d array.
 *
 * [init] is called when rows and cols is known, and [setElement] when element is set.
 */
internal inline fun <reified T> varargEndToArr(
    values: Array<out Any>,
    init: (rows: Int, cols: Int) -> Unit,
    setElement: (r: Int, c: Int, element: T) -> Unit
) {
    val pairs = values.count { it is Pair<*, *> }
    val items = values.size + pairs
    val rows = pairs + 1
    val cols = items / rows
    require(rows * cols == items) { "Even rows/cols not given" }
    init(rows, cols)
    var curRow = 0
    var curCol = 0
    values.forEach {
        when (it) {
            is Pair<*, *> -> {
                val (a, b) = it
                require(a is T && b is T) { "Invalid value given" }
                require(curRow < rows) { "Even cols not given" }
                setElement(curRow, curCol, a)
                curRow++
                setElement(curRow, 0, b)
                curCol = 1
            }
            is T -> {
                require(curCol < cols) { "Even cols not given" }
                setElement(curRow, curCol, it)
                curCol++
            }
            else -> throw IllegalArgumentException("Invalid value given")
        }
    }
}
