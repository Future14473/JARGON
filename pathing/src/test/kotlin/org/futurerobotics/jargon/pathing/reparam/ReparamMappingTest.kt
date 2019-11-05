package org.futurerobotics.jargon.pathing.reparam

import org.futurerobotics.jargon.Debug
import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.function.QuinticSpline
import org.futurerobotics.jargon.util.mapAllPairs
import org.futurerobotics.jargon.util.stepToAll
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class ReparamMappingTest(private val mapping: SamplesReparamMapping, private val allS: List<Double>) {

    @Test
    fun `Single get and stepper are same`() {
        val singleGet = allS.map { mapping.tOfS(it) }
        val bulkGet = mapping.stepToAll(allS)
        Assert.assertTrue("Size differs", bulkGet.size == singleGet.size)

        bulkGet.zip(singleGet).forEachIndexed { index, (first, second) ->
            val b = first == second
            Debug.breakIf(!b)
            if (!b) Assert.fail("Content differs at $index")
        }
    }

    companion object {
        private val random = Random(92367432)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getMappings(): List<Array<Any>> {
            val mappings = List(15) {
                QuinticSpline.random(random, range)
            }.flatMap {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByIntegration(),
                    it.reparamByIntegration(10, 100),
                    it.reparamByIntegration()
                )
            }.map { it.mapping }
            val progressions = List(5) {
                DoubleProgression.fromNumSegments(
                    random.nextDouble(-20.0, 5.0), random.nextDouble(40.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            return mapAllPairs(mappings, progressions).map { arrayOf(it.first, it.second) }
        }
    }
}
