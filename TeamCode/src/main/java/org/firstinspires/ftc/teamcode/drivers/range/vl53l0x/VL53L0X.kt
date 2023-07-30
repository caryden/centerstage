package org.firstinspires.ftc.teamcode.drivers.range.vl53l0x

import android.util.Log
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cWaitControl
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.TypeConversion
import kotlin.experimental.and
import kotlin.experimental.or
import kotlin.math.pow
import org.firstinspires.ftc.teamcode.drivers.utils.*

// Built from guidance derived from:
// https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver
// Used AdaFruit's Python Library as the basis for the register definitions and code:
// https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X/blob/main/adafruit_vl53l0x.py
// Here is another helpful resource:
// https://www.artfulbytes.com/vl53l0x-post
@I2cDeviceType
@DeviceProperties(name = "VL53L0X ToF Laser Ranging Sensor", xmlTag = "VL53L0X")
class VL53L0X(deviceClient: I2cDeviceSynch?, deviceClientIsOwned: Boolean) :
    I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned) {
    companion object {
        val VL53L0X_ADDRESS_DEFAULT : I2cAddr = I2cAddr.create7bit(0x29)

        val SYSRANGE_START = 0x00.toByte()
        val SYSTEM_THRESH_HIGH = 0x0C.toByte()
        val SYSTEM_THRESH_LOW = 0x0E.toByte()
        val SYSTEM_SEQUENCE_CONFIG = 0x01.toByte()
        val SYSTEM_RANGE_CONFIG = 0x09.toByte()
        val SYSTEM_INTERMEASUREMENT_PERIOD = 0x04.toByte()
        val SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A.toByte()
        val GPIO_HV_MUX_ACTIVE_HIGH = 0x84.toByte()
        val SYSTEM_INTERRUPT_CLEAR = 0x0B.toByte()
        val RESULT_INTERRUPT_STATUS = 0x13.toByte()
        val RESULT_RANGE_STATUS = 0x14.toByte()
        val RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC.toByte()
        val RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0.toByte()
        val RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0.toByte()
        val RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4.toByte()
        val RESULT_PEAK_SIGNAL_RATE_REF = 0xB6.toByte()
        val ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28
        val I2C_SLAVE_DEVICE_ADDRESS = 0x8A.toByte()
        val MSRC_CONFIG_CONTROL = 0x60.toByte()
        val PRE_RANGE_CONFIG_MIN_SNR = 0x27.toByte()
        val PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56.toByte()
        val PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57.toByte()
        val PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64.toByte()
        val FINAL_RANGE_CONFIG_MIN_SNR = 0x67.toByte()
        val FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47.toByte()
        val FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48.toByte()
        val FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44.toByte()
        val PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61.toByte()
        val PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62.toByte()
        val PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50.toByte()
        val PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51.toByte()
        val PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52.toByte()
        val SYSTEM_HISTOGRAM_BIN = 0x81.toByte()
        val HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33.toByte()
        val HISTOGRAM_CONFIG_READOUT_CTRL = 0x55.toByte()
        val FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70.toByte()
        val FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71.toByte()
        val FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72.toByte()
        val CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20.toByte()
        val MSRC_CONFIG_TIMEOUT_MACROP = 0x46.toByte()
        val SOFT_RESET_GO2_SOFT_RESET_N = 0xBF.toByte()
        val IDENTIFICATION_MODEL_ID = 0xC0.toByte()
        val IDENTIFICATION_REVISION_ID = 0xC2.toByte()
        val OSC_CALIBRATE_VAL = 0xF8.toByte()
        val GLOBAL_CONFIG_VCSEL_WIDTH = 0x32.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4.toByte()
        val GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5.toByte()
        val GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6.toByte()
        val DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E.toByte()
        val DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F.toByte()
        val POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80.toByte()
        val VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89.toByte()
        val ALGO_PHASECAL_LIM = 0x30.toByte()
        val ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30.toByte()
        val VCSEL_PERIOD_PRE_RANGE = 0
        val VCSEL_PERIOD_FINAL_RANGE = 1
    }
    private var stopVariable: Byte? = null
    init {
        deviceClient!!.i2cAddress = VL53L0X_ADDRESS_DEFAULT

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
        return "VL53L0X Time-of-Flight (ToF) Laser Ranging Sensor"
    }

    override fun doInitialize(): Boolean {
        return if (validateI2CCommunication()) {
            Log.i("VL53L0X", "VL53L0X i2c communication validated successfully")
            initializeSensor()
            true
        } else {
            false
        }
    }
    private fun initializeSensor() {
        Log.i("VL53L0X", "Initializing sensor")

        // Initialize access to the sensor. This is based on the logic from:
        // https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        // Set I2C standard mode.
        val pairs1 = arrayOf(
            Pair(0x88.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte())
        )

        for (pair in pairs1) {
            writeByte(pair.first, pair.second)
        }
         stopVariable = readByte(0x91.toByte())
        val pairs2 = arrayOf(
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x00.toByte())
        )
        for (pair in pairs2) {
            writeByte(pair.first, pair.second)
        }

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4)
        // limit checks
        val configControl: Byte = readByte(MSRC_CONFIG_CONTROL) or 0x12.toByte()
        writeByte(MSRC_CONFIG_CONTROL, configControl)

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        val signalRateLimit = 0.25
        writeByte(SYSTEM_SEQUENCE_CONFIG, 0xFF.toByte())

        val (spadCount, spadIsAperture) = getSpadInfo()
        Log.i("VL53L0X", "spadCount: $spadCount, spadIsAperture: $spadIsAperture")

        // The SPAD map (RefGoodSpadMap) is read by
        // VL53L0X_get_info_from_device() in the API, but the same data seems to
        // be more easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through
        // _6, so read it from there.
        val refSpadMapSize = 6
        val refSpadMap = deviceClient.read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0.toInt(), refSpadMapSize)

        val pairs3 = arrayOf(
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00.toByte()),
            Pair(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4.toByte())
        )

        for (pair in pairs3) {
            writeByte(pair.first, pair.second)
        }

        val firstSpadToEnable: Int = if (spadIsAperture) 12 else 0
        var spadsEnabled = 0

        for (i in 0 until 48) {
            if (i < firstSpadToEnable || spadsEnabled == spadCount) {
                // This bit is lower than the first one that should be enabled,
                // or (reference_spad_count) bits have already been enabled, so
                // zero this bit.
                refSpadMap[(i / 8)] = refSpadMap[(i / 8)] and (1 shl (i % 8)).inv().toByte()
            } else if (refSpadMap[(i / 8)] shr (i % 8) and 0x1 > 0) {
                spadsEnabled++
            }
        }
        deviceClient.write(GLOBAL_CONFIG_SPAD_ENABLES_REF_0.toInt(), refSpadMap)

        // the rest of the pairs to be written
        val remainingPairs = arrayOf<Pair<Byte, Byte>>(
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x09.toByte(), 0x00.toByte()),
            Pair(0x10.toByte(), 0x00.toByte()),
            Pair(0x11.toByte(), 0x00.toByte()),
            Pair(0x24.toByte(), 0x01.toByte()),
            Pair(0x25.toByte(), 0xFF.toByte()),
            Pair(0x75.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x4E.toByte(), 0x2C.toByte()),
            Pair(0x48.toByte(), 0x00.toByte()),
            Pair(0x30.toByte(), 0x20.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x30.toByte(), 0x09.toByte()),
            Pair(0x54.toByte(), 0x00.toByte()),
            Pair(0x31.toByte(), 0x04.toByte()),
            Pair(0x32.toByte(), 0x03.toByte()),
            Pair(0x40.toByte(), 0x83.toByte()),
            Pair(0x46.toByte(), 0x25.toByte()),
            Pair(0x60.toByte(), 0x00.toByte()),
            Pair(0x27.toByte(), 0x00.toByte()),
            Pair(0x50.toByte(), 0x06.toByte()),
            Pair(0x51.toByte(), 0x00.toByte()),
            Pair(0x52.toByte(), 0x96.toByte()),
            Pair(0x56.toByte(), 0x08.toByte()),
            Pair(0x57.toByte(), 0x30.toByte()),
            Pair(0x61.toByte(), 0x00.toByte()),
            Pair(0x62.toByte(), 0x00.toByte()),
            Pair(0x64.toByte(), 0x00.toByte()),
            Pair(0x65.toByte(), 0x00.toByte()),
            Pair(0x66.toByte(), 0xA0.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x22.toByte(), 0x32.toByte()),
            Pair(0x47.toByte(), 0x14.toByte()),
            Pair(0x49.toByte(), 0xFF.toByte()),
            Pair(0x4A.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x7A.toByte(), 0x0A.toByte()),
            Pair(0x7B.toByte(), 0x00.toByte()),
            Pair(0x78.toByte(), 0x21.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x23.toByte(), 0x34.toByte()),
            Pair(0x42.toByte(), 0x00.toByte()),
            Pair(0x44.toByte(), 0xFF.toByte()),
            Pair(0x45.toByte(), 0x26.toByte()),
            Pair(0x46.toByte(), 0x05.toByte()),
            Pair(0x40.toByte(), 0x40.toByte()),
            Pair(0x0E.toByte(), 0x06.toByte()),
            Pair(0x20.toByte(), 0x1A.toByte()),
            Pair(0x43.toByte(), 0x40.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x34.toByte(), 0x03.toByte()),
            Pair(0x35.toByte(), 0x44.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x31.toByte(), 0x04.toByte()),
            Pair(0x4B.toByte(), 0x09.toByte()),
            Pair(0x4C.toByte(), 0x05.toByte()),
            Pair(0x4D.toByte(), 0x04.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x44.toByte(), 0x00.toByte()),
            Pair(0x45.toByte(), 0x20.toByte()),
            Pair(0x47.toByte(), 0x08.toByte()),
            Pair(0x48.toByte(), 0x28.toByte()),
            Pair(0x67.toByte(), 0x00.toByte()),
            Pair(0x70.toByte(), 0x04.toByte()),
            Pair(0x71.toByte(), 0x01.toByte()),
            Pair(0x72.toByte(), 0xFE.toByte()),
            Pair(0x76.toByte(), 0x00.toByte()),
            Pair(0x77.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x0D.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0x01.toByte(), 0xF8.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x8E.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x00.toByte())
        )
        for (pair in remainingPairs) {
            writeByte(pair.first, pair.second)
        }

        writeByte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04.toByte())
        val gpioHvMuxActiveHigh: Byte = readByte(GPIO_HV_MUX_ACTIVE_HIGH)
        writeByte(GPIO_HV_MUX_ACTIVE_HIGH, (gpioHvMuxActiveHigh.toInt() and 0x10.inv()).toByte())  // active low
        writeByte(SYSTEM_INTERRUPT_CLEAR, 0x01.toByte())

        val measurementTimingBudgetUs = measurementTimingBudget
        writeByte(SYSTEM_SEQUENCE_CONFIG, 0xE8.toByte())
        measurementTimingBudget = measurementTimingBudgetUs

        writeByte(SYSTEM_SEQUENCE_CONFIG, 0x01.toByte())
        performSingleRefCalibration(0x40.toByte())

        writeByte(SYSTEM_SEQUENCE_CONFIG, 0x02.toByte())
        performSingleRefCalibration(0x00.toByte())

        // "restore the previous Sequence Config"
        writeByte(SYSTEM_SEQUENCE_CONFIG, 0xE8.toByte())


    }
    var continuousMode = false
        set(value : Boolean) {
            if(value)
                startContinuous()
            else
                stopContinuous()
            field = value
        }

    val range: Short
        get() {
            // Perform a single (or continuous if `startContinuous` called)
            // reading of the range for an object in front of the sensor and
            // return the distance in millimeters.

            // Adapted from readRangeSingleMillimeters in pololu code at:
            //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
            if (!continuousMode) {
                doRangeMeasurement()
            }
            return readRange()
        }

    var measurementTimingBudget : Int = 33000
        get() {
            var budgetUs = 1910 + 960  // Start overhead + end overhead.
            val (tcc, dss, msrc, preRange, finalRange) = getSequenceStepEnables()
            val stepTimeouts = getSequenceStepTimeouts(preRange)
            val (msrcDssTccUs, preRangeUs, finalRangeUs) = stepTimeouts
            if (tcc) {
                budgetUs += msrcDssTccUs + 590
            }
            if (dss) {
                budgetUs += 2 * (msrcDssTccUs + 690)
            } else if (msrc) {
                budgetUs += msrcDssTccUs + 660
            }
            if (preRange) {
                budgetUs += preRangeUs + 660
            }
            if (finalRange) {
                budgetUs += finalRangeUs + 550
            }
            return budgetUs
        }
        set(value) {
            require(value >= 20000)
            var usedBudgetUs = 1320 + 960  // Start (diff from get) + end overhead
            val sequenceStepEnables = getSequenceStepEnables()
            val stepTimeouts = getSequenceStepTimeouts(sequenceStepEnables.preRange)
            val (msrcDssTccUs, preRangeUs, _, finalRangeVcselPeriodPclks, preRangeMclks) = stepTimeouts
            if (sequenceStepEnables.tcc) {
                usedBudgetUs += msrcDssTccUs + 590
            }
            if (sequenceStepEnables.dss) {
                usedBudgetUs += 2 * (msrcDssTccUs + 690)
            } else if (sequenceStepEnables.msrc) {
                usedBudgetUs += msrcDssTccUs + 660
            }
            if (sequenceStepEnables.preRange) {
                usedBudgetUs += preRangeUs + 660
            }
            if (sequenceStepEnables.finalRange) {
                usedBudgetUs += 550
                // "Note that the final range timeout is determined by the timing
                // budget and the sum of all other timeouts within the sequence.
                // If there is no room for the final range timeout, then an error
                // will be set. Otherwise, the remaining time will be applied to
                // the final range."
                if (usedBudgetUs > value) {
                    throw IllegalArgumentException("Requested timeout too big.")
                }
                val finalRangeTimeoutUs = value - usedBudgetUs
                var finalRangeTimeoutMclks = timeoutMicrosecondsToMclks(finalRangeTimeoutUs, finalRangeVcselPeriodPclks)
                if (sequenceStepEnables.preRange) {
                    finalRangeTimeoutMclks += preRangeMclks
                }
                writeShort(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,encodeTimeout(finalRangeTimeoutMclks))
                field = value
            }
        }


    private fun startContinuous() {
        // Perform a continuous reading of the range for an object in front of
        // the sensor.

        // Adapted from startContinuous in pololu code at:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        val pairs = listOf(
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte()),
            Pair(0x91.toByte(), stopVariable!!),
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x00.toByte()),
            Pair(SYSRANGE_START, 0x02.toByte())
        )

        for (pair in pairs) {
            writeByte(pair.first, pair.second)
        }

        val start = System.currentTimeMillis()
        while (readByte(SYSRANGE_START).toInt() and 0x01 > 0) {
            checkForTimeout(start,"startContinuous")
        }
    }
    private fun stopContinuous() {
        // Stop continuous readings.

        // Adapted from stopContinuous in pololu code at:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        val pairs = listOf(
            Pair(SYSRANGE_START, 0x01.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte()),
            Pair(0x91.toByte(), 0x00.toByte()),
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte())
        )

        for (pair in pairs) {
            writeByte(pair.first, pair.second)
        }

        // Restore the sensor to single ranging mode
        doRangeMeasurement()
    }
    private fun doRangeMeasurement() {
        // Perform a single reading of the range for an object in front of the
        // sensor, but without return the distance.

        // Adapted from readRangeSingleMillimeters in pololu code at:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        val pairs = listOf(
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte()),
            Pair(0x91.toByte(), stopVariable!!),
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x00.toByte()),
            Pair(SYSRANGE_START, 0x01.toByte())
        )

        for (pair in pairs) {
            writeByte(pair.first, pair.second)
        }

        val start = System.currentTimeMillis()
        while (readByte(SYSRANGE_START).toInt() and 0x01 > 0)
            checkForTimeout(start, "doRangeMeasurement")
    }

    private fun readRange(): Short {
        // Return a range reading in millimeters.

        // Note: Avoid calling this directly. If you do single mode, you need
        // to call `doRangeMeasurement` first. Or your program will stuck or
        // timeout occurred.

        // Adapted from readRangeContinuousMillimeters in pololu code at:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        val start = System.currentTimeMillis()
        while (!isDataReady)
            checkForTimeout(start, "readRange")

        // Assumptions: Linearity Corrective Gain is 1000 (default)
        // fractional ranging is not enabled
        val rangeMm = readShort((RESULT_RANGE_STATUS + 10).toByte())
        writeByte(SYSTEM_INTERRUPT_CLEAR, 0x01.toByte())
        isDataReady = false
        return rangeMm
    }
    private var isDataReady: Boolean = false
        get() {
            // Check if data is available from the sensor. If true a call to .range
            // will return quickly. If false, calls to .range will wait for the sensor's
            // next reading to be available.
            if (!field) {
                field = readByte(RESULT_INTERRUPT_STATUS).toInt() and 0x07 != 0
            }
            return field
        }
        private set

    private fun performSingleRefCalibration(vhvInitByte: Byte) {
        // based on VL53L0X_perform_single_ref_calibration() from ST API.
        writeByte(SYSRANGE_START, (0x01 or (vhvInitByte.toInt() and 0xFF)).toByte())
        val start = System.currentTimeMillis()
        while ((readByte(RESULT_INTERRUPT_STATUS).toInt() and 0x07) == 0)
            checkForTimeout(start, "performSingleRefCalibration")

        writeByte(SYSTEM_INTERRUPT_CLEAR, 0x01.toByte())
        writeByte(SYSRANGE_START, 0x00.toByte())
    }

    private fun getSequenceStepEnables(): SequenceStepEnables {
        // based on VL53L0X_GetSequenceStepEnables() from ST API
        val sequenceConfig = readByte(SYSTEM_SEQUENCE_CONFIG)
        val tcc = (sequenceConfig shr 4 and 0x1.toByte()) > 0
        val dss = (sequenceConfig shr 3 and 0x1.toByte()) > 0
        val msrc = (sequenceConfig shr 2 and 0x1.toByte()) > 0
        val preRange = (sequenceConfig shr 6 and 0x1.toByte()) > 0
        val finalRange = (sequenceConfig shr 7 and 0x1.toByte()) > 0
        return SequenceStepEnables(tcc, dss, msrc, preRange, finalRange)
    }
    private fun getSequenceStepTimeouts(preRange: Boolean): SequenceStepTimeouts {
        // based on get_sequence_step_timeout() from ST API but modified by
        // pololu here:
        //   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
        val preRangeVcselPeriodPclks = getVcselPulsePeriod(VCSEL_PERIOD_PRE_RANGE)
        val msrcDssTccMclks = ((readByte(MSRC_CONFIG_TIMEOUT_MACROP) + 1) and 0xff).toInt()
        val msrcDssTccUs = timeoutMclksToMicroseconds(msrcDssTccMclks, preRangeVcselPeriodPclks)
        val preRangeMclks = decodeTimeout(readShort(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        val preRangeUs = timeoutMclksToMicroseconds(preRangeMclks, preRangeVcselPeriodPclks)
        val finalRangeVcselPeriodPclks = getVcselPulsePeriod(VCSEL_PERIOD_FINAL_RANGE)
        var finalRangeMclks = decodeTimeout(readShort(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if (preRange) {
            finalRangeMclks -= preRangeMclks
        }
        val finalRangeUs = timeoutMclksToMicroseconds(finalRangeMclks, finalRangeVcselPeriodPclks)
        return SequenceStepTimeouts(msrcDssTccUs, preRangeUs, finalRangeUs, finalRangeVcselPeriodPclks, preRangeMclks)
    }

//        These functions convert between the measurement clock periods (MCLKs) and microseconds,
//        given the VCSEL period in PCLKs (pre-scaler clock periods).
//        The conversions account for the fact that one MCLK period is 2304 VCSEL periods long and
//        one VCSEL period is 1655 pre-scaler periods long. These relationships are used to calculate the length
//        of one MCLK period in nanoseconds (macroPeriodNs).
//        The timeout periods are then converted between MCLKs and microseconds using this value.
    private fun timeoutMclksToMicroseconds(timeoutPeriodMclks: Int, vcselPeriodPclks: Int): Int {
        val macroPeriodNs = ((2304.0 * vcselPeriodPclks * 1655.0) + 500.0) / 1000.0
        return (((timeoutPeriodMclks * macroPeriodNs) + (macroPeriodNs / 2.0)) / 1000.0).toInt()
    }
    private fun timeoutMicrosecondsToMclks(timeoutPeriodUs: Int, vcselPeriodPclks: Int): Int {
        val macroPeriodNs = ((2304.0 * vcselPeriodPclks * 1655.0) + 500.0) / 1000.0
        return (((timeoutPeriodUs * 1000.0) + (macroPeriodNs / 2.0)) / macroPeriodNs).toInt()
    }

    // This function reads a register from the device to get the pulse period for either the pre-range or final range step of the measurement,
    // depending on the input argument. It then increments the value, makes sure it is a byte (0-255), and shifts it left by one bit.
    // If the input argument is neither VCSEL_PERIOD_PRE_RANGE nor VCSEL_PERIOD_FINAL_RANGE, the function returns 255.
    private fun getVcselPulsePeriod(vcselPeriodType: Int): Int {
        when (vcselPeriodType) {
            VCSEL_PERIOD_PRE_RANGE -> {
                val value = readByte(PRE_RANGE_CONFIG_VCSEL_PERIOD)
                return ((value + 1) and 0xFF) shl 1
            }
            VCSEL_PERIOD_FINAL_RANGE -> {
                val value = readByte(FINAL_RANGE_CONFIG_VCSEL_PERIOD)
                return ((value + 1) and 0xFF) shl 1
            }
            else -> return 255
        }
    }
    //    The decodeTimeout function takes an integer representing a timeout value and decomposes it into least significant byte (LSByte)
    //    and most significant byte (MSByte) components. The timeout value is computed using the formula
    //    (LSByte × 2^MSByte) + 1.
    //
    //    The encodeTimeout function performs the opposite operation, taking a timeout value in measurement clock cycles (MCLKs)
    //    and encoding it into a format that can be used to set the sensor's timeout registers. The input timeout value is first
    //    converted into an integer and truncated to 16 bits. Then, the least significant byte and the most significant byte of
    //    the resulting value are computed by iteratively right-shifting the least significant byte until it is less than or equal to 255,
    //    incrementing the most significant byte each time. Finally, the most significant byte and the least significant byte are combined
    //    into a single 16-bit integer which is returned.
    private fun decodeTimeout(value: Short): Int {
        // format: "(LSByte * 2^MSByte) + 1"
        val lsByte = (value and 0x00FF.toShort()).toInt()
        val msByte = (value and 0xFF00.toShort()).toInt()
        val exponent = msByte.shr(8).toInt()
        return lsByte * 2.0.pow(exponent).toInt() + 1
    }
    private fun encodeTimeout(timeoutMclks: Int): Short {
        // format: "(LSByte * 2^MSByte) + 1"
        val timeoutMclksInt = timeoutMclks and 0xFFFF
        var lsByte = 0.toShort()
        var msByte = 0.toShort()
        if (timeoutMclksInt > 0) {
            lsByte = ((timeoutMclksInt - 1) and 0xFF).toShort()
            while (lsByte > 255) {
                lsByte = lsByte shr 1
                msByte++
            }
            return ((msByte shl 8) or (lsByte and 0xFF))
        }
        return 0
    }

    fun getSpadInfo(): Pair<Int, Boolean> {
        // Get reference SPAD count and type, returned as a Pair of count and Boolean isAperture. Based on code from:
        // https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp

        val pairs1 = arrayOf(
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x06.toByte())
        )

        for (pair in pairs1) {
            writeByte(pair.first, pair.second)
        }

        writeByte(0x83.toByte(), readByte(0x83.toByte()) or 0x04.toByte())

        val pairs2 = arrayOf(
            Pair(0xFF.toByte(), 0x07.toByte()),
            Pair(0x81.toByte(), 0x01.toByte()),
            Pair(0x80.toByte(), 0x01.toByte()),
            Pair(0x94.toByte(), 0x6B.toByte()),
            Pair(0x83.toByte(), 0x00.toByte())
        )

        for (pair in pairs2) {
            writeByte(pair.first, pair.second)
        }

        val start = System.currentTimeMillis()
        while (readByte(0x83.toByte()) == 0x00.toByte())
            checkForTimeout(start,"getSpadInfo")

        writeByte(0x83.toByte(), 0x01.toByte())
        val tmp: Byte = readByte(0x92.toByte())
        val count = tmp.toInt() and 0x7F
        val isAperture = (tmp.toInt().shr(7) and 0x01) == 1

        val pairs3 = arrayOf(
            Pair(0x81.toByte(), 0x00.toByte()),
            Pair(0xFF.toByte(), 0x06.toByte())
        )

        for (pair in pairs3) {
            writeByte(pair.first, pair.second)
        }

        writeByte(0x83.toByte(), readByte(0x83.toByte()) and 0xFB.toByte())

        val pairs4 = arrayOf(
            Pair(0xFF.toByte(), 0x01.toByte()),
            Pair(0x00.toByte(), 0x01.toByte()),
            Pair(0xFF.toByte(), 0x00.toByte()),
            Pair(0x80.toByte(), 0x00.toByte())
        )

        for (pair in pairs4) {
            writeByte(pair.first, pair.second)
        }

        return Pair(count, isAperture)
    }
    private fun checkForTimeout(start : Long, method : String, ioTimeOutInMs : Long = 10000) {
        if (ioTimeOutInMs > 0 && (System.currentTimeMillis()  - start) >= ioTimeOutInMs) {
            Log.e("VL53L0X", "$method: i2c Timeout")
            throw RuntimeException("$method: Timeout waiting for VL53L0X!")
        }
    }
    private fun writeShort(address: Byte, value: Short) {
        deviceClient.write(address.toInt(), TypeConversion.shortToByteArray(value), I2cWaitControl.WRITTEN)
    }

    private fun readShort(address: Byte): Short {
        return TypeConversion.byteArrayToShort(deviceClient.read(address.toInt(), 2))
    }
    private fun readByte(address: Byte): Byte {
        return deviceClient.read(address.toInt(), 1)[0]
    }
    private fun writeByte(address: Byte, value: Byte) {
        deviceClient.write(address.toInt(), byteArrayOf(value), I2cWaitControl.WRITTEN)
    }

    private fun validateI2CCommunication(): Boolean {
        // Map of register addresses to expected values, these are from the datasheet
        // https://www.st.com/resource/en/datasheet/vl53l0x.pdf section 3.2 page 19

        val referenceRegisters8Bit = mapOf<Byte, Byte>(
            0xC0.toByte() to 0xEE.toByte(),
            0xC1.toByte() to 0xAA.toByte(),
            0xC2.toByte() to 0x10.toByte()
        )

        // Validate 8-bit registers
        for ((address, expectedValue) in referenceRegisters8Bit) {
            val readValue = readByte(address)
            if (readValue != expectedValue) {
                return false
            }
        }
        return true
    }

}