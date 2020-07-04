package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.junit.jupiter.api.Test

//integration test, I guess
class StateSpaceDesignTest {


    @OptIn(ExperimentalStateSpace::class)
    @Test
    fun `sanity check`() {

        val matrices = ContinuousStateSpaceMatrices(
            matOf(1.7, 2.1, 3 to 1, -3, 1 to -1, -2, 0),
            matOf(1, 1, 1, 1 to -1, -1, 1, 1 to 2, 2, 1, -4),
            idenMat(3) * 0.5
        )
        val lqrCost = QRCost(idenMat(3) * 1.2, idenMat(4) * 2.0)

        val noiseCovariance = NoiseCovariance(idenMat(3), idenMat(4))

        val discMatrices = discretizeZeroOrderHold(matrices, 1 / 20.0)

        val kGain = discreteLQR(discMatrices, lqrCost)

        val runner: StateSpaceRunner = StateSpaceRunnerBuilder()
            .setMatrices(discMatrices)
            .addGainController(kGain)
            .addKalmanFilter {
                setNoiseCovariance(noiseCovariance)
                setInitialProcessCovariance(idenMat(3))
            }
            .addReferenceTracking(plantInversion(discMatrices, null))

            .build()
        runner.reset(Vec(3))
        repeat(10) {
            runner.update(Vec(4), Vec(3), null, 0L)
        }
    }
}
