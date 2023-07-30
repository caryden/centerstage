package org.firstinspires.ftc.teamcode.drivers.range.vl53l1x

import android.util.Log
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cWaitControl
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.TypeConversion
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.drivers.range.vl53l0x.VL53L0X
import org.firstinspires.ftc.teamcode.drivers.utils.shl
import org.firstinspires.ftc.teamcode.drivers.utils.shr
import java.io.Closeable
import java.util.concurrent.atomic.AtomicInteger
import kotlin.experimental.and

// Built from guidance derived from:
// https://github.com/FIRST-Tech-Challenge/ftcrobotcontroller/wiki/Writing-an-I2C-Driver
// https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html#documentation
//
// TODO: add calibration support, add ROI support
@I2cDeviceType
@DeviceProperties(name = "VL53L1X ToF Laser Ranging Sensor", xmlTag = "VL53L1X")
class VL53L1X(deviceClient: I2cDeviceSynch?, deviceClientIsOwned: Boolean) :
    I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned), Closeable {
    companion object {
        private val VL53L1X_ADDRESS_DEFAULT : I2cAddr = I2cAddr.create7bit(0x29)
        private val SOFT_RESET = 0x0000.toShort()
        private val VL53L1_I2C_SLAVE__DEVICE_ADDRESS = 0x0001.toShort()
        private val VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008.toShort()
        private val ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS = 0x0016.toShort()
        private val ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS = 0x0018.toShort()
        private val ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS = 0x001A.toShort()
        private val ALGO__PART_TO_PART_RANGE_OFFSET_MM = 0x001E.toShort()
        private val MM_CONFIG__INNER_OFFSET_MM = 0x0020.toShort()
        private val MM_CONFIG__OUTER_OFFSET_MM = 0x0022.toShort()
        private val DEFAULT_CONFIG_START_INDEX = 0x002D.toShort()
        private val GPIO_HV_MUX__CTRL = 0x0030.toShort()
        private val GPIO__TIO_HV_STATUS = 0x0031.toShort()
        private val SYSTEM__INTERRUPT_CONFIG_GPIO = 0x0046.toShort()
        private val PHASECAL_CONFIG__TIMEOUT_MACROP = 0x004B.toShort()
        private val RANGE_CONFIG__TIMEOUT_MACROP_A_HI = 0x005E.toShort()
        private val RANGE_CONFIG__VCSEL_PERIOD_A = 0x0060.toShort()
        private val RANGE_CONFIG__VCSEL_PERIOD_B = 0x0063.toShort()
        private val RANGE_CONFIG__TIMEOUT_MACROP_B_HI = 0x0061.toShort()
        private val RANGE_CONFIG__TIMEOUT_MACROP_B_LO = 0x0062.toShort()
        private val RANGE_CONFIG__SIGMA_THRESH = 0x0064.toShort()
        private val RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066.toShort()
        private val RANGE_CONFIG__VALID_PHASE_HIGH = 0x0069.toShort()
        private val VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD = 0x006C.toShort()
        private val SYSTEM__THRESH_HIGH = 0x0072.toShort()
        private val SYSTEM__THRESH_LOW = 0x0074.toShort()
        private val SD_CONFIG__WOI_SD0 = 0x0078.toShort()
        private val SD_CONFIG__INITIAL_PHASE_SD0 = 0x007A.toShort()
        private val ROI_CONFIG__USER_ROI_CENTRE_SPAD = 0x007F.toShort()
        private val ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080.toShort()
        private val SYSTEM__SEQUENCE_CONFIG = 0x0081.toShort()
        private val VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD = 0x0082.toShort()
        private val SYSTEM__INTERRUPT_CLEAR = 0x0086.toShort()
        private val SYSTEM__MODE_START = 0x0087.toShort()
        private val VL53L1_RESULT__RANGE_STATUS = 0x0089.toShort()
        private val VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 = 0x008C.toShort()
        private val RESULT__AMBIENT_COUNT_RATE_MCPS_SD = 0x0090.toShort()
        private val VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096.toShort()
        private val VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098.toShort()
        private val VL53L1_RESULT__OSC_CALIBRATE_VAL = 0x00DE.toShort()
        private val VL53L1_FIRMWARE__SYSTEM_STATUS = 0x00E5.toShort()
        private val VL53L1_IDENTIFICATION_MODEL_ID = 0x010F.toShort()
        private val VL53L1_IDENTIFICATION_MODEL_TYPE = 0x0110.toShort()
        private val VL53L1_IDENTIFICATION_MASK_REVISION = 0x0111.toShort()
        private val VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD = 0x013E.toShort()
        private val VL53L1_EXPECTED_MODEL_ID = 0xEA.toByte()
        private val VL53L1_EXPECTED_MODEL_TYPE = 0xCC.toByte()
        private val VL53L1_EXPECTED_MASK_REVISION = 0x10.toByte()

        private val VL53L1X_DEFAULT_CONFIGURATION = byteArrayOf(
            0x00.toByte(), // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
            0x00.toByte(), // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
            0x00.toByte(), // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
            0x01.toByte(), // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()
            0x02.toByte(), // 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady()
            0x00.toByte(), // 0x32 : not user-modifiable
            0x02.toByte(), // 0x33 : not user-modifiable
            0x08.toByte(), // 0x34 : not user-modifiable
            0x00.toByte(), // 0x35 : not user-modifiable
            0x08.toByte(), // 0x36 : not user-modifiable
            0x10.toByte(), // 0x37 : not user-modifiable
            0x01.toByte(), // 0x38 : not user-modifiable
            0x01.toByte(), // 0x39 : not user-modifiable
            0x00.toByte(), // 0x3a : not user-modifiable
            0x00.toByte(), // 0x3b : not user-modifiable
            0x00.toByte(), // 0x3c : not user-modifiable
            0x00.toByte(), // 0x3d : not user-modifiable
            0xff.toByte(), // 0x3e : not user-modifiable
            0x00.toByte(), // 0x3f : not user-modifiable
            0x0F.toByte(), // 0x40 : not user-modifiable
            0x00.toByte(), // 0x41 : not user-modifiable
            0x00.toByte(), // 0x42 : not user-modifiable
            0x00.toByte(), // 0x43 : not user-modifiable
            0x00.toByte(), // 0x44 : not user-modifiable
            0x00.toByte(), // 0x45 : not user-modifiable
            0x20.toByte(), // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
            0x0b.toByte(), // 0x47 : not user-modifiable
            0x00.toByte(), // 0x48 : not user-modifiable
            0x00.toByte(), // 0x49 : not user-modifiable
            0x02.toByte(), // 0x4a : not user-modifiable
            0x0a.toByte(), // 0x4b : not user-modifiable
            0x21.toByte(), // 0x4c : not user-modifiable
            0x00.toByte(), // 0x4d : not user-modifiable
            0x00.toByte(), // 0x4e : not user-modifiable
            0x05.toByte(), // 0x4f : not user-modifiable
            0x00.toByte(), // 0x50 : not user-modifiable
            0x00.toByte(), // 0x51 : not user-modifiable
            0x00.toByte(), // 0x52 : not user-modifiable
            0x00.toByte(), // 0x53 : not user-modifiable
            0xc8.toByte(), // 0x54 : not user-modifiable
            0x00.toByte(), // 0x55 : not user-modifiable
            0x00.toByte(), // 0x56 : not user-modifiable
            0x38.toByte(), // 0x57 : not user-modifiable
            0xff.toByte(), // 0x58 : not user-modifiable
            0x01.toByte(), // 0x59 : not user-modifiable
            0x00.toByte(), // 0x5a : not user-modifiable
            0x08.toByte(), // 0x5b : not user-modifiable
            0x00.toByte(), // 0x5c : not user-modifiable
            0x00.toByte(), // 0x5d : not user-modifiable
            0x01.toByte(), // 0x5e : not user-modifiable
            0xcc.toByte(), // 0x5f : not user-modifiable
            0x0f.toByte(), // 0x60 : not user-modifiable
            0x01.toByte(), // 0x61 : not user-modifiable
            0xf1.toByte(), // 0x62 : not user-modifiable
            0x0d.toByte(), // 0x63 : not user-modifiable
            0x01.toByte(), // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm
            0x68.toByte(), // 0x65 : Sigma threshold LSB
            0x00.toByte(), // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold()
            0x80.toByte(), // 0x67 : Min count Rate LSB
            0x08.toByte(), // 0x68 : not user-modifiable
            0xb8.toByte(), // 0x69 : not user-modifiable
            0x00.toByte(), // 0x6a : not user-modifiable
            0x00.toByte(), // 0x6b : not user-modifiable
            0x00.toByte(), // 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs()
            0x00.toByte(), // 0x6d : Intermeasurement period
            0x0f.toByte(), // 0x6e : Intermeasurement period
            0x89.toByte(), // 0x6f : Intermeasurement period LSB
            0x00.toByte(), // 0x70 : not user-modifiable
            0x00.toByte(), // 0x71 : not user-modifiable
            0x00.toByte(), // 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetDistanceThreshold()
            0x00.toByte(), // 0x73 : distance threshold high LSB
            0x00.toByte(), // 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetDistanceThreshold()
            0x00.toByte(), // 0x75 : distance threshold low LSB
            0x00.toByte(), // 0x76 : not user-modifiable
            0x01.toByte(), // 0x77 : not user-modifiable
            0x0f.toByte(), // 0x78 : not user-modifiable
            0x0d.toByte(), // 0x79 : not user-modifiable
            0x0e.toByte(), // 0x7a : not user-modifiable
            0x0e.toByte(), // 0x7b : not user-modifiable
            0x00.toByte(), // 0x7c : not user-modifiable
            0x00.toByte(), // 0x7d : not user-modifiable
            0x02.toByte(), // 0x7e : not user-modifiable
            0xc7.toByte(), // 0x7f : ROI center, use SetROI()
            0xff.toByte(), // 0x80 : XY ROI (X=Width, Y=Height), use SetROI()
            0x9B.toByte(), // 0x81 : not user-modifiable
            0x00.toByte(), // 0x82 : not user-modifiable
            0x00.toByte(), // 0x83 : not user-modifiable
            0x00.toByte(), // 0x84 : not user-modifiable
            0x01.toByte(), // 0x85 : not user-modifiable
            0x00.toByte(), // 0x86 : clear interrupt, use ClearInterrupt()
            0x00.toByte()  // 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87
        )
    }
    private var currentRange = AtomicInteger(0)
    private var getRangeJob : Job? = null
    private var hasContinuousRangingBeenStarted = false

    init {
        deviceClient!!.i2cAddress = VL53L1X_ADDRESS_DEFAULT

        // Once everything has been set up, we need to engage the sensor to start communicating.
        // We also need to run the arming state callback method that deals with situations involving USB cables disconnecting and reconnecting
        super.registerArmingStateCallback(false);

        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }
    fun getRange() : Int {
        return currentRange.get()
    }
    fun stop() {
        if (hasContinuousRangingBeenStarted)
            runBlocking {
                stopRanging()
                getRangeJob?.cancelAndJoin()
                hasContinuousRangingBeenStarted = false
            }
    }
    override fun close() {
        stop()
    }
    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Other
    }

    override fun getDeviceName(): String {
        return "VL53L1X ToF Laser Ranging Sensor"
    }
    override fun doInitialize(): Boolean {
        return if (validateBootupAndI2CCommunication()) {
            Log.i("VL53L1X", "i2c communication validated and sensor booted up")
            initializeSensor()
            true
        } else false
    }

    private fun validateBootupAndI2CCommunication(): Boolean {
        // first wait for sensor to boot up
        do {
            val sensorState = readByte(VL53L1_FIRMWARE__SYSTEM_STATUS)
        } while (sensorState == 0.toByte())

        // Map of register addresses to expected values, these are from the datasheet
        // https://www.st.com/resource/en/datasheet/vl53l1x.pdf section 4.2 page 22
        val referenceRegisters8Bit = mapOf<Short, Byte>(
            VL53L1_IDENTIFICATION_MODEL_ID to VL53L1_EXPECTED_MODEL_ID,
            VL53L1_IDENTIFICATION_MODEL_TYPE to VL53L1_EXPECTED_MODEL_TYPE
        )
        // Validate 8-bit registers
        for ((index, expectedValue) in referenceRegisters8Bit) {
            val readValue = readByte(index)
            if (readValue != expectedValue)
                return false
        }
        return true
    }
    private fun initializeSensor() {
        runBlocking {
            writeBytes(DEFAULT_CONFIG_START_INDEX, VL53L1X_DEFAULT_CONFIGURATION)

            // we just start and stop ranging here to get the sensor to initialize
            startRanging()
            while(!isDataReady()) // TODO: Add a timeout here
                delay(1)
            clearInterrupt()
            stopRanging()

            writeByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x09) // two bounds VHV
            writeByte(0x0B, 0) // start VHV from the previous temperature
        }
    }
    fun start() : Closeable {
        if(!hasContinuousRangingBeenStarted) {

            startRanging()

            getRangeJob = CoroutineScope(Dispatchers.IO).launch {
                val measurementTime = ElapsedTime()
                while (this.isActive) {

                    // TODO: Add a timeout here
                    while(!isDataReady())
                        delay(1)

                    val range = getDistance()
                    currentRange.set(range.toInt())

                    clearInterrupt()

                    val rangingIntervalMs = measurementTime.milliseconds()
                    measurementTime.reset()

                    delay(rangingIntervalMs.toLong() / 2) // sample at half the measurement interval (2x frequency)
                }
            }
            hasContinuousRangingBeenStarted = true
        }
        return this
    }
    private fun startRanging() {
        writeByte(SYSTEM__MODE_START, 0x40)
    }
    private fun stopRanging() {
        writeByte(SYSTEM__MODE_START, 0x00)
    }
    private fun clearInterrupt() {
        writeByte(SYSTEM__INTERRUPT_CLEAR, 0x01)
    }
    private fun isDataReady() : Boolean {
        val interruptPolarity = getInterruptPolarity()
        val interruptStatus = 	readByte(GPIO__TIO_HV_STATUS);
        val bitMask = 0x01.toByte()
        return when(interruptPolarity) {
            InterruptPolarity.ACTIVE_HIGH -> interruptStatus and bitMask == bitMask
            InterruptPolarity.ACTIVE_LOW -> interruptStatus and bitMask == 0.toByte()
            }
    }

    private fun getInterruptPolarity(): InterruptPolarity {
     return if(readByte(GPIO_HV_MUX__CTRL) and 0x10.toByte() == 0x10.toByte()) InterruptPolarity.ACTIVE_HIGH else InterruptPolarity.ACTIVE_LOW
    }
    /**
     * Function to get the distance mode of the VL53L1X sensor.
     * @return DistanceMode enum indicating the current distance mode.
     */
    private fun getDistanceMode(): DistanceMode {
        val dmRawValue = readByte(PHASECAL_CONFIG__TIMEOUT_MACROP)
        return when (dmRawValue) {
            0x14.toByte() -> DistanceMode.SHORT // DM = 1 in API documentation
            0x0A.toByte() -> DistanceMode.LONG  // DM = 2 in API documentation
            else -> throw IllegalStateException("Unknown distance mode value read from sensor $dmRawValue")
        }
    }
    private fun setDistanceMode(distanceMode: DistanceMode) {
        val timingBudgetInMs = getTimingBudgetInMs()

        when(distanceMode) {
            DistanceMode.SHORT -> {
                writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
                writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
                writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
                writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
                writeShort(SD_CONFIG__WOI_SD0, 0x0705);
                writeShort(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
            }
            DistanceMode.LONG -> {
                writeByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
                writeByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
                writeByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
                writeByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8.toByte())
                writeShort(SD_CONFIG__WOI_SD0, 0x0F0D);
                writeShort(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
            }
            else -> throw IllegalStateException("Unknown distance mode value $distanceMode")
        }

        setTimingBudgetInMs(timingBudgetInMs)
    }
    private fun getTimingBudgetInMs() : Short {
        val rawTimingBudget = readShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI).toInt()
        val timingBudgetInMs = when(rawTimingBudget) {
            0x001D -> 15
            0x0051, 0x001E -> 20
            0x00D6, 0x0060 -> 33
            0x1AE, 0x00AD -> 50
            0x02E1, 0x01CC -> 100
            0x03E1, 0x02D9 -> 200
            0x0591, 0x048F -> 500
            else -> throw IllegalStateException("Unknown timing budget value read from sensor $rawTimingBudget")
        }
        return timingBudgetInMs.toShort()
    }
    private fun setTimingBudgetInMs(timingBudgetInMs : Short) {
        val distanceMode = getDistanceMode()
        when(distanceMode) {
            DistanceMode.SHORT -> {
                when(timingBudgetInMs.toInt()) {
                    15 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001D.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027.toShort())
                    }
                    20 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E.toShort())
                    }
                    33 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E.toShort())
                    }
                    50 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01AE.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8.toShort())
                    }
                    100 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388.toShort())
                    }
                    200 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496.toShort())
                    }
                    500 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1.toShort())
                    }
                    else -> throw IllegalArgumentException("for DistanceMode = $distanceMode, Timing budget must be one of 15, 20, 33, 50, 100, 200, or 500")

                }
            }
            DistanceMode.LONG -> {
                when(timingBudgetInMs.toInt()) {
                    20 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022.toShort())
                    }
                    33 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E.toShort())
                    }
                    50 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6.toShort())
                    }
                    100 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA.toShort())
                    }
                    200 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8.toShort())
                    }
                    500 -> {
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F.toShort())
                        writeShort(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4.toShort())
                    }
                    else -> throw IllegalArgumentException("for DistanceMode = $distanceMode, Timing budget must be one of 20, 33, 50, 100, 200, or 500")
                }
            }
            else -> throw IllegalStateException("Unknown distance mode value read from sensor: $distanceMode")
        }
    }
    private fun setInterMeasurementInMs(interMeasurementMs : Int) {
        val clockPLLMask = 0x3FF
        val clockPLL = readShort(VL53L1_RESULT__OSC_CALIBRATE_VAL).toInt() and clockPLLMask
        val systemIntermeasurementPeriod = (clockPLL * interMeasurementMs * 1.075).toInt()
        writeInt(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,systemIntermeasurementPeriod);
    }
    private fun getInterMeasurementInMs() : Int {
        val clockPLLMask = 0x3FF
        val clockPLL = readShort(VL53L1_RESULT__OSC_CALIBRATE_VAL).toInt() and clockPLLMask
        val systemIntermeasurementPeriod = readInt(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD)
        return (systemIntermeasurementPeriod / (clockPLL * 1.075)).toInt()
    }
    private fun getDistance() : Short {
       return readShort(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
    }
    private fun setOffset(offset : Short) {
        val shiftedOffset = offset.toInt() * 4
        writeShort(ALGO__PART_TO_PART_RANGE_OFFSET_MM,shiftedOffset.toShort())
        writeShort(MM_CONFIG__INNER_OFFSET_MM, 0x0)
        writeShort(MM_CONFIG__OUTER_OFFSET_MM, 0x0)
    }
    private fun getOffset() : Short {
        val rawOffset = readShort(ALGO__PART_TO_PART_RANGE_OFFSET_MM)
        val shiftedOffset = (rawOffset shl 3) shr 5
        return if(shiftedOffset > 1024) (shiftedOffset - 2048).toShort() else shiftedOffset
    }
    private fun readByte(index : Short): Byte {
        deviceClient.write(TypeConversion.shortToByteArray(index), I2cWaitControl.WRITTEN)
        return deviceClient.read(1)[0]
    }
    private fun readShort(index : Short): Short {
        deviceClient.write(TypeConversion.shortToByteArray(index), I2cWaitControl.WRITTEN)
        return TypeConversion.byteArrayToShort(deviceClient.read(2))
    }
    private fun readInt(index : Short): Int {
        deviceClient.write(TypeConversion.shortToByteArray(index), I2cWaitControl.WRITTEN)
        return TypeConversion.byteArrayToInt(deviceClient.read(4))
    }

    private fun writeInt(index : Short, value: Int, waitControl: I2cWaitControl = I2cWaitControl.WRITTEN) {
        deviceClient.write( TypeConversion.shortToByteArray(index) + TypeConversion.intToByteArray(value), waitControl)
    }
    private fun writeShort(index : Short, value: Short, waitControl: I2cWaitControl = I2cWaitControl.WRITTEN) {
        deviceClient.write( TypeConversion.shortToByteArray(index) + TypeConversion.shortToByteArray(value), waitControl)
    }
    private fun writeByte(index : Short, value: Byte, waitControl: I2cWaitControl = I2cWaitControl.WRITTEN) {
        deviceClient.write( TypeConversion.shortToByteArray(index) + byteArrayOf(value), waitControl)
    }
    private fun writeBytes(index : Short, values: ByteArray, waitControl: I2cWaitControl = I2cWaitControl.WRITTEN) {
        deviceClient.write(TypeConversion.shortToByteArray(index) + values, waitControl)
    }

}