package org.futurerobotics.jargon.linalg

/**
 * Converts vararg elements in the style of ``(a,b to c,d)`` to a 2d array like structure.
 *
 * [init] is called when the number of rows and cols is known (to create the array), and [setElement] to set the value
 * in the array.
 *
 * @param E element type
 * @param A array type
 */
internal inline fun <reified E, A> varargBuilderToArray(
    values: Array<out Any>,
    init: (rows: Int, cols: Int) -> A,
    setElement: A.(r: Int, c: Int, element: E) -> Unit
): A {
    //find size and create the array
    val pairs = values.count { it is Pair<*, *> }
    val items = values.size + pairs
    val rows = pairs + 1
    val cols = items / rows
    require(rows * cols == items) { "Even rows/cols not given" }
    val array = init(rows, cols)

    //fill the array
    var curRow = 0
    var curCol = 0
    values.forEach {
        when (it) {
            is Pair<*, *> -> {
                //pair indicates row end
                val (a, b) = it
                require(a is E) { "Invalid value $a given" }
                require(b is E) { "Invalid value $b given" }
                require(curRow < rows) { "Even rows/cols not given" }
                array.setElement(curRow, curCol, a)
                curRow++
                array.setElement(curRow, 0, b)
                curCol = 1
            }
            is E -> {
                require(curCol < cols) { "Even rows/cols not given" }
                array.setElement(curRow, curCol, it)
                curCol++
            }
            else -> throw IllegalArgumentException("Invalid value of $it given")
        }
    }
    return array
}
