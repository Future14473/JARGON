package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.Debug
import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.TAU
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
internal class PathTest(private val path: Path, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val singleGet = allS.map { path.atLength(it) }
        val bulkGet = path.stepToAll(allS)
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
            val curves = MutableList(2) {
                QuinticSpline.random(random, range)
            }.flatMap {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByIntegration(),
                    it.reparamByIntegration(10, 100),
                    it.reparamByIntegration()
                )
            }.let {
                it + List(2) {
                    Line(random.nextVector2d(range), random.nextVector2d(range))
                }
            }
            val headings = MutableList(2) {
                listOf(
                    OffsetTangentHeading(random.nextDouble(TAU)),
                    ConstantHeading(random.nextDouble(10_000.0)),
                    LinearInterpolatedHeading(
                        random.nextDouble(
                            -1000.0, 1000.0
                        ), random.nextDouble(-1000.0, 1000.0)
                    )
                )
            }.also { it += listOf(TangentHeading) }.flatten()
            val progressions = List(2) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            val rawPaths = allPairs(curves, headings)
                .mapTo(ArrayList()) { it.first.addHeading(it.second) }
            rawPaths.shuffle()
            val paths = rawPaths.chunked(4) { MultiplePath(it, checkContinuity = false) }
            return allPairs(paths, progressions).map { arrayOf(it.first, it.second) }
        }
    }
}
