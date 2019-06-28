package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.TAU
import org.futurerobotics.temporaryname.math.function.nextQuinticSpline
import org.futurerobotics.temporaryname.math.nextVector2d
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByArcSubdivisions
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.util.allPairs
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class PathsSegmentsTest(private val path: PathSegment, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val bulkGet = path.getAllPointInfo(allS)
        val singleGet = allS.map { path.getPointInfo(it) }
        assertTrue(bulkGet.zip(singleGet).all { it.first contentEquals it.second })
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            val curves = MutableList(2) { random.nextQuinticSpline(range) }.flatMap {
                listOf(
                    it.reparamByIntegration(),
                    it.reparamByArcSubdivisions(),
                    it.reparamByIntegration(10, 100),
                    it.reparamByArcSubdivisions(2.0, 1.0, 0.2)
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
                    LinearInterpolatedHeading(random.nextDouble(-1000.0, 1000.0), random.nextDouble(-1000.0, 1000.0))
                )
            }.also { it += listOf(TangentHeading) }.flatten()
            val progressions = List(4) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            val paths = allPairs(curves, headings).map { it.first.addHeading(it.second) }
            return allPairs(paths, progressions).map { arrayOf(it.first, it.second) }
        }
    }
}