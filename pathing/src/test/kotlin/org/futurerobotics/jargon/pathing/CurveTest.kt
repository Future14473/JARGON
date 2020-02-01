package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.Debug
import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.nextVector2d
import org.futurerobotics.jargon.util.mapAllPairs
import org.futurerobotics.jargon.util.stepToAll
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class CurveTest(private val curve: Curve, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val singleGet = allS.map { curve.pointAt(it) }
        val bulkGet = curve.stepToAll(allS)
        Assert.assertTrue("Size differs", bulkGet.size == singleGet.size)

        bulkGet.zip(singleGet).forEachIndexed { index, (first, second) ->
            val b = first contentEquals second
            Debug.breakIf(!b)
            if (!b) Assert.fail("Content differs at $index")
        }
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            val curves = MutableList(5) {
                randomQuinticSpline(random, range)
            }.flatMapTo(ArrayList<Curve>()) {
                listOf(
                    it.reparameterizeToCurve(),
                    it.reparameterizeToCurve(
                        IntegrationReparameterizer(
                            100,
                            20
                        )
                    )
                )
            }.also {
                repeat(2) { _ ->
                    it += Line(
                        random.nextVector2d(range), random.nextVector2d(
                            range
                        )
                    )
                }
            }
            val progressions = List(10) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            return mapAllPairs(curves, progressions) { c, p -> arrayOf(c, p) }
        }
    }
}
