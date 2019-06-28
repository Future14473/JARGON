package org.futurerobotics.temporaryname.pathing.path

import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.TAU
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.math.function.nextQuinticSpline
import org.futurerobotics.temporaryname.math.randomVectorDerivatives
import org.futurerobotics.temporaryname.pathing.path.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.util.allPairs
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class PathsTest(private val path: Path, private val allS: List<Double>) {

    @Test
    fun `Bulk get (getAllPointInfo) and single get (getPointInfo) are same`() {
        val bulkGet = path.getAllPointInfo(allS)
        val singleGet = allS.map { path.getPointInfo(it) }
        Assert.assertTrue(bulkGet.size == singleGet.size)
        Assert.assertTrue(bulkGet.zip(singleGet).all { it.first contentEquals it.second })
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0
        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            val paths = List(10) { randomPath(20, TangentHeading) } + List(10) {
                Path(
                    random.nextQuinticSpline(range).reparamByIntegration().addHeading( //not continuous, but thats besides the point.
                        LinearInterpolatedHeading(
                            random.nextDouble(TAU), random.nextDouble(2344444.0)
                        )
                    )
                )
            }

            val progressions = List(20) {
                DoubleProgression.fromNumSegments(
                    random.nextDouble(-4.0, 0.0), random.nextDouble(4.0, 400.0), random.nextInt(40, 10000)
                ).toList()
            }
            return allPairs(paths, progressions).map { arrayOf(it.first, it.second) }
        }

        private fun randomPath(numPoints: Int, headingProvider: HeadingProvider): Path {
            val pts = List(numPoints) {
                randomVectorDerivatives(random, range)
            }
            val segs = pts.zipWithNext { a, b ->
                QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(headingProvider)
            }
            return Path(segs)
        }
    }
}