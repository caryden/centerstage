package org.firstinspires.ftc.teamcode.drivers.range.vl53l1x

data class RangingResult(val range : Int, val roi : ROI = VL53L1X.VL53L1X_DEFAULT_ROI, val sequence : Int = 0) {
    override fun toString(): String {
        return "RangingResult(sequence=$sequence range=$range mm, roi=${roi.name})"
    }
}
