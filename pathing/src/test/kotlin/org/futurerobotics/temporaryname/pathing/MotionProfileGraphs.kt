@file:Suppress("ClassName")

package org.futurerobotics.temporaryname.pathing

import org.futurerobotics.temporaryname.math.DoubleProgression
import org.futurerobotics.temporaryname.math.function.QuinticSpline
import org.futurerobotics.temporaryname.math.randomVectorDerivatives
import org.futurerobotics.temporaryname.pathing.reparam.reparamByIntegration
import org.futurerobotics.temporaryname.pathing.trajectory.*
import org.futurerobotics.temporaryname.profile.MotionProfile
import org.futurerobotics.temporaryname.profile.MotionProfileGenerator
import org.futurerobotics.temporaryname.saveGraph
import org.futurerobotics.temporaryname.util.stepToAll
import org.junit.Assume
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.junit.runners.Parameterized
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.Styler
import org.knowm.xchart.style.markers.None
import kotlin.random.Random
import kotlin.system.measureNanoTime

@RunWith(Parameterized::class)
class MotionProfileGraphs(
    private val path: Path,
    private val constraints: MotionConstraintSet,
    private val pathNumber: Int,
    private val profileNumber: Int
) {

    private val xs = DoubleProgression.fromNumSegments(0.0, path.length, 1000).toList()
    private var generationTime: Long = 0L
    private lateinit var profile: MotionProfile
    @Before
    fun generateAndTimeProfile() {
        generationTime = measureNanoTime {
            val constraints = TrajectoryConstraint(path, constraints)
            profile = MotionProfileGenerator.generateDynamicProfile(
                constraints, path.length, targetStartVel = 0.0, targetEndVel = 0.0, segmentSize = 0.01
            )
        }
    }

    @Test
    fun `Report generation time`() {
        val millis = generationTime / 1e6
        println("Profile Generation took $millis millis")
        val ratio = millis / path.length
        println("Generation-length ratio: $ratio")
    }

    @Test
    fun `Generate profile graph`() {
        val points = path.stepToAll(xs)
        //profile
        val chart: XYChart = with(XYChartBuilder()) {
            title("Motion Profile and Constraints")
            xAxisTitle("Arc length")
            yAxisTitle("Velocity")
            height(600)
            width(1200)
        }.build()
        chart.styler.apply {
            yAxisMin = 0.0
            yAxisMax = yMax
            legendPosition = Styler.LegendPosition.OutsideS
            legendLayout = Styler.LegendLayout.Horizontal
        }
        constraints.velocityConstraints.forEach { velConst ->
            val constraintVelocities = points.map(velConst::maxVelocity)
            val name = velConst.toString()
            chart.addSeries(name, xs, constraintVelocities).apply {
                marker = None()
                lineWidth = 1f
            }
        }
        val profileSpeeds = xs.map { profile.atDistance(it).v }
        chart.addSeries("Final Profile", xs, profileSpeeds).apply {
            marker = None()
            lineWidth = 1f
        }
        chart.saveGraph("Motion Profile and Constraints/$pathNumber/Profile$profileNumber")
    }

    @Test
    fun `Generate path graph`() {
        Assume.assumeTrue(profileNumber == 0)
        val pathChart: XYChart = with(XYChartBuilder()) {
            title("Path")
            xAxisTitle("x")
            yAxisTitle("y")
            height(600)
            width(800)
        }.build()
        val pts = path.stepToAll(xs).map { it.position }
        pathChart.styler.apply {
            //            isToolTipsEnabled = true
            //            isToolTipsAlwaysVisible = true
            //            toolTipBorderColor = Color(1, 1, 1, 0)
            //            toolTipBackgroundColor = Color(1, 1, 1, 0)
            legendPosition = Styler.LegendPosition.OutsideS
            legendLayout = Styler.LegendLayout.Horizontal
        }
        pathChart.addSeries("The path", pts.map { it.x }, pts.map { it.y }).apply {
            marker = None()
            //            toolTips = xs.map { "%.2f".format(it) }.toTypedArray()
        }
        pathChart.saveGraph("Motion Profile and Constraints/$pathNumber/Path")
    }

    companion object {
        private val random = Random(234875422L)
        private const val range = 20.0
        private const val yMax = 5.5
        private val constantConstraints = mutableListOf(
            MotionConstraintSet(
                MaxVelocityConstraint(5.0),
                MaxPathAngularVelocityConstraint(1.5),
                MaxCentripetalAccelConstraint(0.9),
                MaxTangentAccelConstraint(0.9),
                MaxTotalAccelerationConstraint(1.0),
                MaxAngularAccelerationConstraint(0.3)
            )
        ).also {
            it += List(2) {
                randomConstraints()
            }
        }

        private fun randomConstraints(): MotionConstraintSet {
            return MotionConstraintSet(
                MaxVelocityConstraint(random.nextDouble(3.0, 5.0)),
                MaxPathAngularVelocityConstraint(
                    random.nextDouble(
                        0.3,
                        3.0
                    )
                ),
                MaxCentripetalAccelConstraint(
                    random.nextDouble(
                        1.0,
                        3.0
                    )
                ),
                MaxTangentAccelConstraint(
                    random.nextDouble(
                        1.0,
                        3.0
                    )
                ),
                MaxTotalAccelerationConstraint(
                    random.nextDouble(
                        1.0,
                        3.0
                    )
                ),
                MaxAngularAccelerationConstraint(
                    random.nextDouble(
                        0.5,
                        2.0
                    )
                )
            )
        }

        private val paths: List<Path> = List(10) {
            var path: Path? = null
            measureNanoTime {
                val segs = List(5) {
                    randomVectorDerivatives(random, range)
                }.zipWithNext { a, b ->
                    QuinticSpline.fromDerivatives(a, b).reparamByIntegration().addHeading(TangentHeading)
                }
                path = MultiplePath(segs)
            }.also { println("Path generation took ${it / 1e6} millis") }
            path!!
        }

        @JvmStatic
        @Parameterized.Parameters
        fun `paths and constraints`(): List<Array<Any>> {
            val list = mutableListOf<Array<Any>>()
            paths.forEachIndexed { pathNum, path ->
                val thisConstraints = constantConstraints + List(2) { randomConstraints() }
                thisConstraints.forEachIndexed { setNum, set ->
                    list += arrayOf(path, set, pathNum, setNum)
                }
            }
            return list
        }
    }
}
