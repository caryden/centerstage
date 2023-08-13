package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import edu.ncssm.ftc.electric_mayhem.i2c.vl53l0x.VL53L0X

@TeleOp
class VL53L0XSimpleRangeTest : LinearOpMode() {
    override fun runOpMode()  {
        val dashboard = FtcDashboard.getInstance()
        val telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        // here we should use the "use" method to ensure that the VL53L0X is closed when we are done with it
        // this way it ends its coroutine
        hardwareMap.get(VL53L0X::class.java, "vl53l0x").use { rangeSensor ->
            val timer = ElapsedTime()

            waitForStart()
            while (opModeIsActive()) {
                timer.reset()
                val range = rangeSensor.getRange()
                val elapsedTime = timer.milliseconds()
                telemetry.addData("Range (mm)", range)
                telemetry.addData("Loop Time (ms)", elapsedTime)
                telemetry.update()
            }
        }
    }
}