package org.futurerobotics.jargon.statespace

import org.futurerobotics.jargon.linalg.*
import org.junit.jupiter.api.Test
import java.util.*

//integration test, I guess
class StateSpaceDesignTest {


    @Test
    fun `sanity check`() {

        val matrices = ContinuousStateSpaceMatrices(
            Mat(1.7, 2.1, 3 to 1, -3, 1 to -1, -2, 0),
            Mat(1, 1, 1, 1 to -1, -1, 1, 1 to 2, 2, 1, -4),
            idenMat(3) * 0.5
        )
        val lqrCost = QRCost(idenMat(3) * 1.2, idenMat(4) * 2.0)

        val noiseCovariance = NoiseCovariance(idenMat(3), idenMat(4))

        val discMatrices = discretize(matrices, 1 / 20.0)

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
        val random = Random()
        runner.reset(normRandVec(3, random))
        repeat(10) {
            runner.update(normRandVec(4, random), normRandVec(3, random), null, 0L)
        }
    }
}
