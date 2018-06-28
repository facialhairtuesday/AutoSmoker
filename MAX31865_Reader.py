import time, math
import RPi.GPIO as GPIO

class max31865(object):
    """
    Reading Temp from the MAX31865 with GPIO
    using RPi.  Any pins can be used
    """
    def __init__(self, csPin, misoPin, mosiPin, clkPin):
        self.csPin = csPin
        self.misoPin = misoPin
        self.mosiPin = mosiPin
        self.clkPin = clkPin
        self.setupGPIO()

    def setupGPIO(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.csPin, GPIO.OUT)
        GPIO.setup(self.misoPin, GPIO.IN)
        GPIO.setup(self.mosiPin, GPIO.OUT)
        GPIO.setup(self.clkPin, GPIO.OUT)

        GPIO.output(self.csPin, GPIO.HIGH)
        GPIO.output(self.clkPin, GPIO.LOW)
        GPIO.output(self.mosiPin, GPIO.LOW)

    def readTemp(self):
        # one shot
        self.writeRegister(0, 0xB2)

        # conversion time is less than 100ms
        time.sleep(.1)  # give it 100ms for conversion

        # read all registers
        out = self.readRegisters(0, 8)

        conf_reg = out[0]
        print "config register byte: %x" % conf_reg

        [rtd_msb, rtd_lsb] = [out[1], out[2]]
        rtd_ADC_Code = ((rtd_msb << 8) | rtd_lsb) >> 1

        temp_C = self.calcPT100Temp(rtd_ADC_Code)

        [hft_msb, hft_lsb] = [out[3], out[4]]
        hft = ((hft_msb << 8) | hft_lsb) >> 1
        print "high fault threshold: %d" % hft

        [lft_msb, lft_lsb] = [out[5], out[6]]
        lft = ((lft_msb << 8) | lft_lsb) >> 1
        print "low fault threshold: %d" % lft

        status = out[7]
        if ((status & 0x80) == 1):
            raise FaultError("High threshold limit (Cable fault/open)")
        if ((status & 0x40) == 1):
            raise FaultError("Low threshold limit (Cable fault/short)")
        if ((status & 0x04) == 1):
            raise FaultError("Overvoltage or Undervoltage Error")

    def writeRegister(self, regNum, dataByte):
        GPIO.output(self.csPin, GPIO.LOW)

        # 0x8x to specify 'write register value'
        addressByte = 0x80 | regNum;

        # first byte is address byte
        self.sendByte(addressByte)
        # the rest are data bytes
        self.sendByte(dataByte)

        GPIO.output(self.csPin, GPIO.HIGH)

    def readRegisters(self, regNumStart, numRegisters):
        out = []
        GPIO.output(self.csPin, GPIO.LOW)

        # 0x to specify 'read register value'
        self.sendByte(regNumStart)

        for byte in range(numRegisters):
            data = self.recvByte()
            out.append(data)

        GPIO.output(self.csPin, GPIO.HIGH)
        return out

    def sendByte(self, byte):
        for bit in range(8):
            GPIO.output(self.clkPin, GPIO.HIGH)
            if (byte & 0x80):
                GPIO.output(self.mosiPin, GPIO.HIGH)
            else:
                GPIO.output(self.mosiPin, GPIO.LOW)
            byte <<= 1
            GPIO.output(self.clkPin, GPIO.LOW)

    def recvByte(self):
        byte = 0x00
        for bit in range(8):
            GPIO.output(self.clkPin, GPIO.HIGH)
            byte <<= 1
            if GPIO.input(self.misoPin):
                byte |= 0x1
            GPIO.output(self.clkPin, GPIO.LOW)
        return byte

    def calcPT100Temp(self, RTD_ADC_Code):
        R_REF = 430.0  # Reference Resistor --> 430 Ohm on Adafruit Board
        Res0 = 100.0;  # Resistance at 0 degC for 400ohm R_Ref
        a = .00390830
        b = -.000000577500
        # c = -4.18301e-12 # for -200 <= T <= 0 (degC)
        c = -0.00000000000418301
        # c = 0 # for 0 <= T <= 850 (degC)
        print "RTD ADC Code: %d" % RTD_ADC_Code
        Res_RTD = (RTD_ADC_Code * R_REF) / 32768.0  # PT100 Resistance
        print "PT100 Resistance: %f ohms" % Res_RTD
        #
        # Callendar-Van Dusen equation
        # Res_RTD = Res0 * (1 + a*T + b*T**2 + c*(T-100)*T**3)
        # Res_RTD = Res0 + a*Res0*T + b*Res0*T**2 # c = 0
        # (c*Res0)T**4 - (c*Res0)*100*T**3
        # + (b*Res0)*T**2 + (a*Res0)*T + (Res0 - Res_RTD) = 0
        #
        # quadratic formula:
        # for 0 <= T <= 850 (degC)
        temp_C = -(a * Res0) + math.sqrt(a * a * Res0 * Res0 - 4 * (b * Res0) * (Res0 - Res_RTD))
        temp_C = temp_C / (2 * (b * Res0))
        temp_C_line = (RTD_ADC_Code / 32.0) - 256.0

        print "Straight Line Approx. Temp: %f degC" % temp_C_line
        print "Callendar-Van Dusen Temp (degC > 0): %f degC" % temp_C
        # print "Solving Full Callendar-Van Dusen using numpy: %f" %  temp_C_numpy
        if (temp_C < 0):  # use straight line approximation if less than 0
            # Can also use python lib numpy to solve cubic
            # Should never get here in this application
            temp_C = (RTD_ADC_Code / 32) - 256
        return temp_C

    class FaultError(Exception):
        pass

    if __name__ == "__main__":
        import max31865
        csPin = 8
        misoPin = 9
        mosiPin = 10
        clkPin = 11
        max = max31865.max31865(csPin, misoPin, mosiPin, clkPin)
        tempC = max.readTemp()
        GPIO.cleanup()