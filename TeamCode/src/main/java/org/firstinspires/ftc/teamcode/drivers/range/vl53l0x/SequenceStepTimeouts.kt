package org.firstinspires.ftc.teamcode.drivers.range.vl53l0x

data class SequenceStepTimeouts(
    val msrcDssTccUs: Int,
    val preRangeUs: Int,
    val finalRangeUs: Int,
    val finalRangeVcselPeriodPclks: Int,
    val preRangeMclks: Int
)
