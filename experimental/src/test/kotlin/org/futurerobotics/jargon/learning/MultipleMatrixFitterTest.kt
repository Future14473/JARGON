package org.futurerobotics.jargon.learning

import org.futurerobotics.jargon.linalg.*
import org.futurerobotics.jargon.saveGraph
import org.junit.jupiter.api.Test
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.style.markers.None
import strikt.api.expectThat
import strikt.assertions.isLessThan
import java.util.*

internal class MultipleMatrixFitterTest {
    private val real = listOf(
        Mat(
            1.2, 2.1 to
                    -3.2, 4.8 to
                    5.3, 2.9
        ), Mat(
            -1.0, 0, -4 to
                    7.2, -1, 2 to
                    3.3, 5, -0.5
        )
    )

    private fun randomInput(random: Random): List<Vec> = real.map {
        normRandVec(it.cols, random)
    }

    @Test
    fun batch() {
        val random = Random("Machine learning is a dangerous word".hashCode().toLong())
        val perfect = BaseMultipleMatrixPredictor(real)
        val predictor = BaseMultipleMatrixPredictor(
            real.map { normRandMat(it.rows, it.cols, random) }
        )
        val fitter =
            BatchMultipleMatrixFitter(10, MultipleDescentParams(listOf(0.01, 0.01), listOf(0.0001, 0.0001)))

        val xTest = List(100) { randomInput(random) }
        val yTest = xTest.map { perfect.predict(it) }

        val numPoints = 100
        val perUpdate = 30
        val costs = List(numPoints) {
            val xTrain = List(perUpdate) { randomInput(random) }
            val yTrain = perfect.predict(xTrain).map { it + normRandVec(it.size, random) * 10.0 }
            fitter.fitOnce(predictor, xTrain, yTrain)

            fitter.cost(predictor, xTest, yTest)
        }
        XYChartBuilder().apply {
            title = "Cost over time"
            xAxisTitle("Iteration")
        }.build().apply {
            addSeries("cost", costs).apply {
                marker = None()
            }
        }.saveGraph("MultipleMatrixFitterTest batch", 300)



        println("Final mats: ${predictor.mats.map { it.formatReadable() }}")

        expectThat(costs.last()).isLessThan(10.0)
    }

    @Test
    fun stochastic() {
        val random = Random("The robot apocolypse is not upon us yet".hashCode().toLong())
        val perfect = BaseMultipleMatrixPredictor(real)
        val predictor = BaseMultipleMatrixPredictor(
            real.map { normRandMat(it.rows, it.cols, random) }
        )
        val fitter =
            StochasticMultipleMatrixFitter(MultipleDescentParams(listOf(0.0025, 0.0025), listOf(0.0001, 0.0001)))

        val xTest = List(100) { randomInput(random) }
        val yTest = xTest.map { perfect.predict(it) }

        val numPoints = 100
        val perUpdate = 30
        val costs = List(numPoints) {
            val xTrain = List(perUpdate) { randomInput(random) }
            val yTrain = perfect.predict(xTrain).map { it + normRandVec(it.size, random) * 10.0 }
            fitter.fitOnce(predictor, xTrain, yTrain)

            fitter.cost(predictor, xTest, yTest)
        }
        XYChartBuilder().apply {
            title = "Cost over time"
            xAxisTitle("Iteration")
        }.build().apply {
            addSeries("cost", costs).apply {
                marker = None()
            }
        }.saveGraph("MultipleMatrixFitterTest stochastic", 300)

        println("Final mats: ${predictor.mats.map { it.formatReadable() }}")

        expectThat(costs.last()).isLessThan(10.0)
    }
}
