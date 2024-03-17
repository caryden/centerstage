package org.firstinspires.ftc.teamcode.opmodes.teleop

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import edu.ncssm.ftc.electricmayhem.devices.range.i2c.vl53l1x.RangingResult
import edu.ncssm.ftc.electricmayhem.devices.range.i2c.vl53l1x.VL53L1X


@TeleOp
class VL53L1XSimpleRangeTest : LinearOpMode() {
    override fun runOpMode()  {
        val dashboard = FtcDashboard.getInstance()
        val telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        // here we should use the "use" method to ensure that the VL53L0X is closed when we are done with it
        // this way it ends its coroutine
        val rangeSensor = hardwareMap.get(VL53L1X::class.java, "vl53l1x")
        val timer = ElapsedTime()
        val zoneList = VL53L1X.FULL_SPREAD_13ZONE_4SPAD_ROI_LIST
        rangeSensor.setRegionsOfInterest(zoneList)
        val map = HashMap<String, RangingResult>(zoneList.size)

        rangeSensor.use {
            rangeSensor.start()

            waitForStart()

            while (opModeIsActive()) {
                timer.reset()
                val rangingResult = rangeSensor.getRangingResult()
                map[rangingResult.roi.name] = rangingResult

                val packet = TelemetryPacket()
                packet.fieldOverlay()
                    .setFill("blue").fillCircle(0.0, 0.0, 1.0)

                for (zone in map.keys) {
                    telemetry.addData(zone, map[zone]!!.range)

                    val rangeInInches = map[zone]!!.range.toDouble() / 25.4
                    val direction = map[zone]!!.roi.direction
                    packet.fieldOverlay()
                        .setFill("blue").fillCircle( rangeInInches * direction.z, rangeInInches * direction.x, 0.5)
                }
                dashboard.sendTelemetryPacket(packet)

                val elapsedTime = timer.milliseconds()
                telemetry.addData("map size", map.values.size)
                telemetry.addData("ROI", rangingResult.roi.name)
                telemetry.addData("Range (mm)", rangingResult.range)
                telemetry.addData("Direction", rangingResult.roi.direction.toString())
                telemetry.addData("Loop Time (ms)", elapsedTime)
                telemetry.update()
            }
            rangeSensor.stop()
        }
    }
}