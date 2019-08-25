package org.futurerobotics.temporaryname.control.controller

import org.futurerobotics.temporaryname.math.Pose2d

class TranslationalAxialPIDController : Controller<Pose2d, Pose2d, List<Double>> {
    override fun process(
        current: Pose2d,
        reference: Pose2d,
        referenceDeriv: Pose2d?,
        elapsedNanos: Long
    ): List<Double> {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun reset() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}