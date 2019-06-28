package org.futurerobotics.temporaryname.pathing.path

infix fun CurvePointInfo.contentEquals(b: CurvePointInfo): Boolean {
    fun CurvePointInfo.getContentArray() = arrayOf(
        position, positionDeriv, positionSecondDeriv, tanAngle, tanAngleDeriv, tanAngleSecondDeriv
    )
    return getContentArray().contentEquals(b.getContentArray())
}

infix fun PathPointInfo.contentEquals(b: PathPointInfo): Boolean {
    fun PathPointInfo.getContentArray() = arrayOf(
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
    return getContentArray().contentEquals(b.getContentArray())
}