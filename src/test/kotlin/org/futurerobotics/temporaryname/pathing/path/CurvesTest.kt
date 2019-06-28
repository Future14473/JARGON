package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.Debug
import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.function.nextQuinticSpline
import org.futurerobotics.temporaryname.math.nextVector2d
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByArcSubdivisions
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.util.allPairs
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class CurvesTest(private val curve: Curve, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val bulkGet = curve.getAllPointInfo(allS)
        val singleGet = allS.map { curve.getPointInfo(it) }
        bulkGet.zip(singleGet).all {
            val b = it.first contentEquals it.second
            Debug.breakIf(!b)
            b
        }.let { assert(it) }
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            val curves = MutableList(5) { random.nextQuinticSpline(range) }.flatMap {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByArcSubdivisions(),
                    it.reparamByIntegration(10, 100),
                    it.reparamByArcSubdivisions(2.0, 1.0, 0.2)
                )
            }.let {
                it + List(5) {
                    Line(random.nextVector2d(range), random.nextVector2d(range))
                }
            }
            val progressions = List(10) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            return allPairs(curves, progressions).map { arrayOf(it.first, it.second) }
        }
    }
}