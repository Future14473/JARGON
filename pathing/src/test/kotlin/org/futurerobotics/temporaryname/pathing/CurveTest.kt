package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.Debug
import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.math.nextVector2d
import org.futurerobotics.temporaryname.pathing.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.util.allPairs
import org.futurerobotics.temporaryname.util.stepToAll
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class CurveTest(private val curve: Curve, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val singleGet = allS.map { curve.atLength(it) }
        val bulkGet = curve.stepToAll(allS)
        Assert.assertTrue("Size differs", bulkGet.size == singleGet.size)

        bulkGet.zip(singleGet).forEachIndexed { index, it ->
            val b = it.first contentEquals it.second
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
            val rawCurves = MutableList(5) {
                QuinticSpline.random(random, range)
            }.flatMapTo(ArrayList<Curve>()) {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByIntegration(100, 20)
                )
            }.also {
                repeat(5) { _ ->
                    it += Line(
                        random.nextVector2d(range), random.nextVector2d(
                            range
                        ))
                }
            }
            val progressions = List(10) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            rawCurves.shuffle()
            val curves = rawCurves.chunked(4) { MultipleCurve(it, checkContinuity = false) }
            return allPairs(curves, progressions) { c, p -> arrayOf(c, p) }
        }
    }
}
