package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivers.range.vl53l0x.VL53L0X

@TeleOp
class VL53L0XContinuousRangeTest : LinearOpMode() {
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        val telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val rangeSensor = hardwareMap.get(VL53L0X::class.java, "vl53l0x")

        // Optionally adjust the measurement timing budget to change speed and accuracy.
        // See the example here for more details:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/examples/Single/Single.ino
        // For example a higher speed but less accurate timing budget of 20ms:
        // vl53.measurementTimingBudget = 20000
        // Or a slower but more accurate timing budget of 200ms:
        //depthSensor.measurementTimingBudget = 33000

        // The default timing budget is 33ms, a good compromise of speed and accuracy.

        // You will see the benefit of continuous mode if you set the measurement timing
        // budget very high, while your program doing something else. When your program done
        // with something else, and the sensor already calculated the distance, the result
        // will return instantly, instead of waiting the sensor measuring first.

        rangeSensor.continuousMode = true
        try {
            waitForStart()
            while (opModeIsActive()) {
                telemetry.addData("Range (mm)", rangeSensor.range)
                telemetry.update()
            }
        }
        finally {
            rangeSensor.continuousMode = false

        }
    }
}