package org.futurerobotics.jargon.learning

//TODO
///**
// * A discrete and online-updatable variant of something like a [MotorVelocityModel].
// *
// *
// * This is also a [MultipleMatrixPredictor] with matrices (A,B,F), so it can be updated with a [StochasticUpdatingBlock]
// * online. We don't say the ML-word.
// *
// * This is also a [DiscreteLinearStateSpaceModel] without the F term; A feed forward of `B^-1*F*sign(x)`
// * can be added to compensate to friction according to this model.
// *
// * This is also [Serializable] so that the model can be stored as a file.
// *
// */
//class UpdatableMotorVelocityModel(
//    override val A: Mat,
//    override val B: Mat,
//    override val F: Mat,
//    override val period: Double
//) : MotorVelDiscreteLinSSModel, MultipleMatrixPredictor, Serializable {
//
//    override val mats: List<Mat> = listOf(A, B, F).asUnmodifiableList()
//
//    override val ySize: Int = mats.fold(-1) { acc, cur ->
//        require(cur.isSquare) { "All matrices must be square, got (${cur.rows} x ${cur.cols})" }
//        if (acc == -1)
//            cur.rows
//        else {
//            require(cur.rows == acc) { "All matrices must have the same size" }
//            acc
//        }
//    }
//
//    override val numInputs: Int get() = mats.size
//    override val xSizes: List<Int> = mats.map { it.cols }.asUnmodifiableList()
//
//    override fun predict(input: List<Vec>): Vec = mats.zip(input, Mat::times).reduce(Vec::add)
//
//    override val stateSize: Int
//        get() = ySize
//    override val inputSize: Int
//        get() = ySize
//    override val outputSize: Int
//        get() = ySize
//
//    override fun getFeedForward(x: Vec): Vec = -B.inv() * F * sign(x)
//
//    override val C: Mat = idenMat(ySize)
//    override val D: Mat = zeroMat(ySize, ySize)
//
//    companion object {
//        private const val serialVersionUID: Long = -88231094383843
//    }
//}
