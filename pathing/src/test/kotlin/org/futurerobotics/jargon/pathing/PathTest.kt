package org.futurerobotics.jargon.pathing

import org.futurerobotics.jargon.math.DoubleProgression
import org.futurerobotics.jargon.math.TAU
import org.futurerobotics.jargon.math.nextVector2d
import org.futurerobotics.jargon.util.stepToAll
import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import strikt.api.expectThat
import kotlin.random.Random

@RunWith(Parameterized::class)
internal class PathTest(private val path: Path, private val allS: List<Double>) {

    @Test
    fun `pointAt and stepTo return the same`() {
        val singleGet = allS.map { path.pointAt(it) }
        val bulkGet = path.stepToAll(allS)
        Assert.assertTrue("Size differs", bulkGet.size == singleGet.size)

        bulkGet.zip(singleGet).forEachIndexed { index, (bulk, single) ->
            expectThat(bulk) {
                assertThat("Matches at $index") { it contentEquals single }
            }
        }
    }

    companion object {
        private val random = Random(2346)
        private const val range = 10.0

        @JvmStatic
        @Parameterized.Parameters
        fun getParams(): List<Array<Any>> {
            val curves = MutableList(2) {
                randomQuinticSpline(random, range)
            }.flatMap {
                listOf(
                    it.reparameterizeToCurve(),
                    it.reparameterizeToCurve(),
                    it.reparameterizeToCurve(
                        IntegrationReparameterizer(
                            10,
                            100
                        )
                    ),
                    it.reparameterizeToCurve()
                )
            }.let {
                it + List(2) {
                    Line(random.nextVector2d(range), random.nextVector2d(range))
                }
            }
            val headings = MutableList(2) {
                listOf(
                    TangentHeading(random.nextDouble(TAU)),
                    ConstantHeading(random.nextDouble(10_000.0)),
                    LinearlyInterpolatedHeading(
                        random.nextDouble(-1000.0, 1000.0),
                        random.nextDouble(-1000.0, 1000.0)
                    )
                )
            }.also { it += listOf(TangentHeading) }.flatten()
            val progressions = List(2) {
                DoubleProgression.fromNumSegments(
                    0.0, random.nextDouble(30.0), random.nextInt(10_000, 80_000)
                ).toList()
            }
            val paths = mapAllPairs(curves, headings) { c, h -> c.addHeading(h) }
            return mapAllPairs(paths, progressions) { p, g -> arrayOf(p, g) }
        }
    }
}
