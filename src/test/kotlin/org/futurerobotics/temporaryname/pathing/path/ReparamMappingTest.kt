package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.function.nextQuinticSpline
import org.futurerobotics.temporaryname.pathing.path.reparam.ReparamMapping
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByArcSubdivisions
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.util.allPairs
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class ReparamMappingTest(private val mapping: ReparamMapping, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllTofS) and single get (tOfS) are same`() {
        val singleGet = allS.map { mapping.tOfS(it) }
        val bulkGet = mapping.getAllTOfS(allS)
        assert(singleGet.zip(bulkGet).all { it.first == it.second })
    }

    companion object {
        private val random = Random(92367432)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getMappings(): List<Array<Any>> {
            val mappings = List(15) { random.nextQuinticSpline(range) }.flatMap {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByArcSubdivisions(),
                    it.reparamByIntegration(10, 100),
                    it.reparamByArcSubdivisions(2.0, 1.0, 0.2)
                )
            }.map { it.mapping }
            val progressions = List(5) {
                DoubleProgression.fromNumSegments(
                    random.nextDouble(-20.0, 5.0), random.nextDouble(40.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            return allPairs(mappings, progressions).map { arrayOf(it.first, it.second) }
        }
    }
}