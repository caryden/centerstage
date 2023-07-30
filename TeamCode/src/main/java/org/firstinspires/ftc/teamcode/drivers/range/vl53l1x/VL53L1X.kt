package org.firstinspires.ftc.teamcode.drivers.range.vl53l1x

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType

// Built from guidance derived from:
// https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver

@I2cDeviceType
@DeviceProperties(name = "VL53L1X ToF Laser Ranging Sensor", xmlTag = "VL53L1X")
class VL53L1X(deviceClient: I2cDeviceSynch?, deviceClientIsOwned: Boolean) :
    I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned) {
    companion object {
        val VL53L1X_ADDRESS_DEFAULT : I2cAddr = I2cAddr.create7bit(0x29)

    }
    init {
        deviceClient!!.i2cAddress = VL53L1X_ADDRESS_DEFAULT

        // Once everything has been set up, we need to engage the sensor to start communicating.
        // We also need to run the arming state callback method that deals with situations involving USB cables disconnecting and reconnecting
        super.registerArmingStateCallback(false);

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Other
    }

    override fun getDeviceName(): String {
        return "VL53L1X ToF Laser Ranging Sensor"
    }
    override fun doInitialize(): Boolean {
        return if (validateI2CCommunication()) {
            initializeSensor()
            true
        } else {
            false
        }
    }
    private fun validateI2CCommunication(): Boolean {
        // Map of register addresses to expected values, these are from the datasheet
        // https://www.st.com/resource/en/datasheet/vl53l1x.pdf section 4.2 page 22
//
//        val referenceRegisters8Bit = mapOf<Int, Byte>(
//            0xC0.toByte() to 0xEE.toByte(),
//            0xC1.toByte() to 0xAA.toByte(),
//            0xC2.toByte() to 0x10.toByte()
//        )
//deviceClient.write()
//        // Validate 8-bit registers
//        for ((address, expectedValue) in referenceRegisters8Bit) {
////            val readValue = readByte(address)
//            if (readValue != expectedValue) {
//                return false
//            }
//        }
        return true
    }

    private fun initializeSensor() {
    }
}