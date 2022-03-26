from wpilib.interfaces import Gyro, Accelerometer
from wpimath.geometry import Rotation2d
import threading
import time
from wpilib import SerialPort
import struct
import logging
import queue

log = logging.getLogger(__name__)


class WIT_IO:
    DATA_BITS = 8
    PARITY_BITS = SerialPort.Parity.kParity_None
    STOP_BITS = SerialPort.StopBits.kStopBits_One
    BUFFER_SIZE = 100
    NUM_OF_BYTES = 11
    FRAME_HEADER = 0x55
    WRITE_BUFFER_SIZE = 100

    # Convert Units
    KAngle = 180.0
    KGyroRate = 2000.0
    KAccel = 16.0

    gyroRates = [0., 0., 0.]
    gyroAngles = [0., 0., 0.]
    accels = [0., 0., 0.]

    byteArray = bytearray(BUFFER_SIZE)
    byteCount = 0

    def __init__(self, serialPort, baud=9600):
        self.imu = SerialPort(baud,
                              serialPort,
                              self.DATA_BITS,
                              self.PARITY_BITS,
                              self.STOP_BITS)

        self.imu.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess)
        self.imu.setWriteBufferSize(self.WRITE_BUFFER_SIZE)
        self._unlock()

    def _parseGyroRate(self, data):
        gyroRates = []

        for i in data[:3]:
            gyroRate = (i) / 32768.0 * self.KGyroRate
            if gyroRate >= self.KGyroRate:
                gyroRate -= 2 * self.KGyroRate
            gyroRates.append(gyroRate)

        self.gyroRates = gyroRates

    def _parseGyroAngle(self, data):
        # Clockwise negative in range [-180, 180]
        angles = []
        for i, a in enumerate(data[:3]):
            angle = (a) / 32768.0 * self.KAngle
            if angle >= self.KAngle:
                angle -= 2 * self.KAngle

            angles.append(angle)

        self.gyroAngles = angles

    def _parseAccels(self, data):
        accels = []

        for i in data[:3]:
            accel = (i) / 32768.0 * self.KAccel
            if accel >= self.KAccel:
                accel -= 2 * self.KAccel
            accels.append(accel)

        self.accels = accels

    def _getData(self, b):
        self.byteArray[self.byteCount] = b
        self.byteCount += 1

        if self.byteCount < self.NUM_OF_BYTES:
            return

        if self.byteArray[0] != self.FRAME_HEADER:
            # Shift the byte array 1 bit left
            for i in range(self.NUM_OF_BYTES - 1):
                self.byteArray[i] = self.byteArray[i + 1]

            self.byteCount -= 1
            return

        if (sum(self.byteArray[:self.NUM_OF_BYTES-1]) & 0xff) != self.byteArray[self.NUM_OF_BYTES - 1]:
            byteRepr = [hex(b) for b in self.byteArray[:self.NUM_OF_BYTES]]
            log.error("Parity error: " + ' '.join(byteRepr))
            self.byteCount = 0
            return

        data = self.byteArray[2:self.NUM_OF_BYTES-1]       # high low bytes
        # concatenate 8 high and low bytes into 4 shorts in little endian
        data = struct.unpack('<hhhh', data)

        if (self.byteArray[1] == 0x51):  # Accelerometer
            self._parseAccels(data)
        elif (self.byteArray[1] == 0x52):  # Gyro Rate
            self._parseGyroRate(data)
        elif (self.byteArray[1] == 0x53):  # Gyro Angle
            self._parseGyroAngle(data)

        self.byteCount = 0

    def _write(self, buf):
        _buf = bytearray(buf)
        self.imu.write(_buf)
        self.imu.flush()
        time.sleep(0.1)

    def _unlock(self):
        # Unlock the IMU so that we can write/set instruction to it
        self._write(b'\xff\xaa\x69\x88\xb5')
        time.sleep(0.1)

    def _calibrate(self):
        self._write(b'\xff\xaa\x01\x01\x00')  # Accelerometer calibration
        self._saveProfile()

        self._write(b'\xff\xaa\x01\x00\x00')  # Finish calibration
        self._saveProfile()

    def _saveProfile(self):
        self._write(b'\xff\xaa\x00\x00\x00')  # Save the current profile

    def _zeroGyroRates(self):
        def mapInt(x): return int(x / self.KGyroRate * 32768)
        rates = map(mapInt, self.gyroRates)
        packed_data = struct.pack('<hhh', *rates)

        self._write(b'\xff\xaa\x08' + packed_data[:2])
        self._saveProfile()

        self._write(b'\xff\xaa\x09' + packed_data[2:4])
        self._saveProfile()

        self._write(b'\xff\xaa\x0a' + packed_data[4:])
        self._saveProfile()

        self.gyroRates = [0.0, 0.0, 0.0]

    def _zeroAccels(self):
        def mapInt(x): return int(x / self.KAccel * 32768)
        accels = map(mapInt, self.accels)
        packed_data = struct.pack('<hhh', *accels)

        self._write(b'\xff\xaa\x05' + packed_data[:2])
        self._saveProfile()

        self._write(b'\xff\xaa\x06' + packed_data[2:4])
        self._saveProfile()

        self._write(b'\xff\xaa\x07' + packed_data[4:])
        self._saveProfile()

        self.accels = [0.0, 0.0, 0.0]

    def _zeroAngles(self):
        self._zeroGyroRates()

        self._write(b'\xff\xaa\x01\x04\x00')  # Reset z angle
        self._saveProfile()
        self.gyroAngles = [0.0, 0.0, 0.0]

    def getData(self):
        # This function should be call periodically to get the latest data from the IMU
        bytesAval = self.imu.getBytesReceived()

        if bytesAval > 0:
            rawReading = bytearray(bytesAval)
            bytesReceived = self.imu.read(rawReading)

            if bytesAval != bytesReceived:
                raise Exception("Did not receive all bytes")

            for b in rawReading:
                self._getData(b)

    def calibrate(self):
        print("Calibrating IMU...")
        self._calibrate()
        time.sleep(3)
        self._zeroAngles()
        self._zeroAccels()

    def getGyroAngles(self):
        return self.gyroAngles

    def getGyroRates(self):
        return self.gyroRates

    def getAccels(self):
        return self.accels

    def zeroAngles(self):
        self._zeroAngles()

    def zeroAccels(self):
        self._zeroAccels()


class WIT_THREAD(threading.Thread):
    def __init__(self, proxiIMU, imuIO, updateRate=50, q=queue.Queue()):
        super().__init__(self)
        self.proxiIMU = proxiIMU
        self.imuIO = imuIO
        self.updateRate = updateRate
        self.q = q

    def run(self):
        while True:
            try:
                function, args, kwargs = self.q.get(timeout=0.02)
                function(*args, **kwargs)
            except queue.Empty:
                self.idle()
            finally:
                self.proxiIMU.update(
                    self.imuIO.getGyroRates(),
                    self.imuIO.getGyroAngles(),
                    self.imuIO.getAccels()
                )

    def idle(self):
        self.imuIO.getData()
        time.sleep(1 / self.updateRate)

    def onThread(self, function, *args, **kwargs):
        self.q.put((function, args, kwargs))


class WitIMU(Gyro):
    gyroRates = [0., 0., 0.]
    gyroAngles = [0., 0., 0.]
    accels = [0., 0., 0.]

    def __init__(self, serialPort):
        super().__init__()
        self.io = WIT_IO(serialPort)
        self.ioThread = WIT_THREAD(self, self.io)

        print(self.ioThread)

        self.ioThread.start()

    def __del__(self):
        if self.ioThread is not None:
            self.ioThread.join(1)

    def update(self, gyroRates, gyroAngles, accels):
        self.gyroRates = gyroRates
        self.gyroAngles = gyroAngles
        self.accels = accels

    def getAngle(self):
        """
        Returns the z angle in degrees.
        """
        return self.gyroAngles[2]

    def getRate(self):
        """
        Returns the z rate in degrees per second.
        """
        return self.gyroRates[2]

    def getX(self):
        return self.accels[0]

    def getY(self):
        return self.accels[1]

    def getZ(self):
        return self.accels[2]

    def setRange(self, range):
        log.warning("This function is intended not implemented!")

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngle())

    def calibrate(self):
        self.io.calibrate()

    def reset(self):
        self.ioThread.onThread(self.ioThread.imuIO.zeroAngles)
        self.ioThread.onThread(self.ioThread.imuIO.zeroAccels)
