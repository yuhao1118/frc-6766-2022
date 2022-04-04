import threading
import time
from wpilib import SerialPort, reportWarning
import struct
import logging

log = logging.getLogger(__name__)


class TOF_IO:
    DATA_BITS = 8
    PARITY_BITS = SerialPort.Parity.kParity_None
    STOP_BITS = SerialPort.StopBits.kStopBits_One
    BUFFER_SIZE = 100
    NUM_OF_BYTES = 16
    FRAME_HEADER = 0x57

    KDistance = 1000.0

    distance = 0.0

    byteArray = bytearray(BUFFER_SIZE)
    byteCount = 0

    def __init__(self, serialPort, baud=115200):
        self.tof = SerialPort(baud,
                              serialPort,
                              self.DATA_BITS,
                              self.PARITY_BITS,
                              self.STOP_BITS)

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

        data = self.byteArray[8:11]
        data = struct.unpack('<I', data + b'\x00')[0]
        self.distance = data / self.KDistance

        self.byteCount = 0

    def getData(self):
        # This function should be call periodically to get the latest data from the TOF
        bytesAvals = self.tof.getBytesReceived()

        if bytesAvals > 0:
            rawReading = bytearray(bytesAvals)
            bytesReceived = self.tof.read(rawReading)

            if bytesAvals != bytesReceived:
                raise Exception("Did not receive all bytes")

            for b in rawReading:
                self._getData(b)

    def getDistance(self):
        return self.distance

class TOF_Thread(threading.Thread):
    def __init__(self, proxyTOF, tofIO, updateRate=25):
        super().__init__(self)
        self.proxyTOF = proxyTOF
        self.tofIO = tofIO
        self.updateRate = updateRate

    def run(self):
        while True:
            self.tofIO.getData()
            self.proxyTOF.update(self.tofIO.getDistance())
            time.sleep(1 / self.updateRate)

class NpTOF():
    distance = 0.0

    io = None
    ioThread = None

    def __init__(self, serialPort):
        try:
            self.io = TOF_IO(serialPort)
            self.ioThread = TOF_Thread(self, self.io)
            self.ioThread.start()
        except:
            log.warning("TOF not found!")
            reportWarning("TOF not found!")

    def __del__(self):
        if self.ioThread is not None:
            self.ioThread.join(1)
        
    def getDistance(self):
        return self.distance

    def update(self, distance):
        self.distance = distance