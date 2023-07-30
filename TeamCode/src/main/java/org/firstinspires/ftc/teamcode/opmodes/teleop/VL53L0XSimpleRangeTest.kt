package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivers.range.vl53l0x.VL53L0X

@TeleOp
class VL53L0XSimpleRangeTest : LinearOpMode() {
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val depthSensor = hardwareMap.get(VL53L0X::class.java, "vl53l0x")

        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Range (mm)", depthSensor.range)
            telemetry.update()
        }
    }
}