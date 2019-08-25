package org.futurerobotics.temporaryname.math

import org.futurerobotics.temporaryname.util.doubleBinarySearch
import org.futurerobotics.temporaryname.util.extendingDoubleSearch
import org.futurerobotics.temporaryname.util.extendingDownDoubleSearch
import org.junit.Test
import kotlin.random.Random

internal class BinarySearchTest {

    @Test
    fun `double binary search`() {
        val random = Random(82487234)
        repeat(1_000_000) {
            val partitionPoint = random.nextDouble(-300.0, 300.0)
            val tolerance = random.nextDouble(0.00001, 0.1)
            val upperExtend = random.nextDouble(300.0)
            val lowerExtend = random.nextDouble(300.0)
            val lower = partitionPoint - lowerExtend
            val upper = partitionPoint + upperExtend
            val binarySearchVal = doubleBinarySearch(
                lower, upper, tolerance
            ) { it > partitionPoint }
            assert(binarySearchVal distTo partitionPoint <= tolerance)
        }
    }

    @Test
    fun `extending binary search`() {
        val random = Random(82487234)
        repeat(1_000_000) {
            val partitionPoint = random.nextDouble(-300.0, 300.0)
            val tolerance = random.nextDouble(0.00001, 0.1)
            val upperExtend = random.nextDouble(300.0)
            val lowerExtend = random.nextDouble(300.0)
            val lower = partitionPoint - lowerExtend
            val upper = partitionPoint + upperExtend
            val binarySearchVal = extendingDoubleSearch(
                lower, upper, tolerance
            ) { it > partitionPoint }
            assert(binarySearchVal distTo partitionPoint <= tolerance)
        }
    }

    @Test
    fun `extending down search`() {
        val random = Random(82487234)
        repeat(1_000_000) {
            val partitionPoint = random.nextDouble(-300.0, 300.0)
            val tolerance = random.nextDouble(0.00001, 0.1)
            val upperExtend = random.nextDouble(300.0)
            val lowerExtend = random.nextDouble(300.0)
            val lower = partitionPoint - lowerExtend
            val upper = partitionPoint + upperExtend
            val binarySearchVal = extendingDownDoubleSearch(
                lower, upper, tolerance
            ) { it > partitionPoint }
            assert(binarySearchVal distTo partitionPoint <= tolerance)
        }
    }
}