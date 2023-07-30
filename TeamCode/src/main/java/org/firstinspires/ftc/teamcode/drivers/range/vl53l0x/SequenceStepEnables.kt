package org.firstinspires.ftc.teamcode.drivers.range.vl53l0x

data class SequenceStepEnables(
    val tcc: Boolean,
    val dss: Boolean,
    val msrc: Boolean,
    val preRange: Boolean,
    val finalRange: Boolean
)
