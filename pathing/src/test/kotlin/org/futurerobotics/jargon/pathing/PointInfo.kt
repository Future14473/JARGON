package org.futurerobotics.jargon.pathing

private fun CurvePoint.getContentArray() = arrayOf(
    position, positionDeriv, positionSecondDeriv, tanAngle, tanAngleDeriv, tanAngleSecondDeriv
)

private fun PathPoint.getContentArray() = arrayOf(
    position,
    positionDeriv,
    positionSecondDeriv,
    tanAngle,
    tanAngleDeriv,
    tanAngleSecondDeriv,
    heading,
    headingDeriv,
    headingSecondDeriv
)

infix fun PathPoint.contentEquals(b: PathPoint): Boolean = getContentArray() contentEquals b.getContentArray()

infix fun CurvePoint.contentEquals(b: CurvePoint): Boolean = getContentArray() contentEquals b.getContentArray()
