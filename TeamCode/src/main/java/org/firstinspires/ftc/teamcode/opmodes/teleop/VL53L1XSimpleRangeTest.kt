package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.drivers.range.vl53l1x.VL53L1X

@TeleOp
class VL53L1XSimpleRangeTest : LinearOpMode() {
    override fun runOpMode()  {
        val dashboard = FtcDashboard.getInstance()
        val telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        // here we should use the "use" method to ensure that the VL53L0X is closed when we are done with it
        // this way it ends its coroutine
        val rangeSensor = hardwareMap.get(VL53L1X::class.java, "vl53l1x")
        val timer = ElapsedTime()

        waitForStart()
        Log.i("VL53L1X", "Starting from OpMode")
        rangeSensor.use {
            rangeSensor.start()
            while (opModeIsActive()) {
                timer.reset()
                val range = rangeSensor.getRange()
                val elapsedTime = timer.milliseconds()
                telemetry.addData("Range (mm)", range)
                telemetry.addData("Loop Time (ms)", elapsedTime)
                telemetry.update()
            }
            rangeSensor.stop()
        }
    }
}