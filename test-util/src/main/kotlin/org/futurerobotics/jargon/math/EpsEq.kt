package org.futurerobotics.jargon.math

import org.futurerobotics.jargon.linalg.*
import strikt.api.Assertion

/** Asserts that this [Double] epsEq another. */
infix fun Assertion.Builder<Double>.isEpsEqTo(expected: Double): Assertion.Builder<Double> =
    assert("is epsEq to %s", expected) {
        if (it epsEq expected) pass()
        else fail(actual = it)
    }

/** Asserts that this [Vector2d] epsEq another. */
infix fun Assertion.Builder<Vector2d>.isEpsEqTo(expected: Vector2d): Assertion.Builder<Vector2d> =
    assert("is epsEq to %s", expected) {
        if (it epsEq expected) pass()
        else fail(actual = it)
    }

/** Asserts that this [Pose2d] epsEq another. */
infix fun Assertion.Builder<Pose2d>.isEpsEqTo(expected: Pose2d): Assertion.Builder<Pose2d> =
    assert("is epsEq to %s", expected) {
        if (it epsEq expected) pass()
        else fail(actual = it)
    }

/** Asserts that this [Mat] epsEq another. */
fun Assertion.Builder<Mat>.isEpsEqTo(expected: Mat, epsilon: Double = EPSILON): Assertion.Builder<Mat> =
    assert("is epsEq to %s", expected) {
        if (it.epsEq(expected, epsilon)) pass()
        else fail(actual = it)
    }


/** Asserts that this [Vec] epsEq another. */
infix fun Assertion.Builder<Vec>.isEpsEqTo(expected: Vec): Assertion.Builder<Vec> =
    assert("is epsEq to %s", expected) {
        if (it epsEq expected) pass()
        else fail(actual = it)
    }
