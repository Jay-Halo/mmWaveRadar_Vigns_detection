# author: kylin, cql22@mails.tsinghua.edu.cn
from pathlib import Path
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import sys
from mmVS.com import serialConfig, parseConfigFile
from mmVS.vital_signs import (
    decode_vital_signs_payload,
    is_vital_signs_tlv,
)

configFileName = str(
    Path(__file__).resolve().parent
    / "profiles"
    / "xwr1843_profile_VitalSigns_20fps.cfg"
)

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0
numRangeBinProcessed = 33 - 11 + 1
Breathsignal = list(range(0, 250))
Heartbeatsignal = list(range(0, 250))
Chestdisplacement = [0] * 250
Rangeprofile = []
Breathenerge = [0] * 250
Heartenerge = [0] * 250


def readAndParseData68xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength, numRangeBinProcessed
    MMWDEMO_UART_MSG_RANGE_PROFILE = 2

    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    magicOK = 0
    dataOK = 0
    frameNumber = 0
    vitalsign = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)

    if (byteBufferLength + byteCount) < len(byteBuffer):
        byteBuffer[byteBufferLength:(byteBufferLength + byteCount)] = byteVec
        byteBufferLength += byteCount

    if byteBufferLength > 16:
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        if startIdx:
            if 0 < startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength - startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0]:]),
                                                                      dtype='uint8')
                byteBufferLength -= startIdx[0]

            if byteBufferLength < 16:
                return dataOK, None, None

            totalPacketLen = int.from_bytes(byteBuffer[12:16], byteorder='little')
            if (
                40 <= totalPacketLen <= len(byteBuffer)
                and byteBufferLength >= totalPacketLen
            ):
                magicOK = 1

    if magicOK:
        idX = 0
        idX += 8
        idX += 4
        totalPacketLen = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        platform = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        frameNumber = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        idX += 4
        vitalsign["numDetectedObj"] = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        numTLVs = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        idX += 4

        for _ in range(numTLVs):
            if idX + 8 > totalPacketLen:
                break
            tlv_type = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4
            tlv_length = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4
            payload_start = idX
            payload_end = payload_start + tlv_length
            if payload_end > totalPacketLen:
                break

            if is_vital_signs_tlv(tlv_type, tlv_length):
                payload = memoryview(byteBuffer[payload_start:payload_end])
                vitalsign.update(decode_vital_signs_payload(payload))
                dataOK = 1

            elif tlv_type == MMWDEMO_UART_MSG_RANGE_PROFILE:
                if vitalsign.__contains__("rangeBinEndIndex"):
                    numRangeBinProcessed = vitalsign["rangeBinEndIndex"] - vitalsign["rangeBinStartIndex"] + 1

                vitalsign["RangeProfile"] = []
                if platform == 0xA1843:
                    # xWR18xx OOB range profile is one uint16 magnitude per bin.
                    numRangeBinProcessed = tlv_length // 2
                    for offset in range(payload_start, payload_end, 2):
                        vitalsign["RangeProfile"].append(
                            int.from_bytes(byteBuffer[offset:offset + 2], "little")
                        )
                else:
                    # Legacy vital-signs demo exports complex int16 samples.
                    sample_count = min(numRangeBinProcessed, tlv_length // 4)
                    for sample_index in range(sample_count):
                        offset = payload_start + sample_index * 4
                        real = int.from_bytes(
                            byteBuffer[offset:offset + 2], "little", signed=True
                        )
                        imag = int.from_bytes(
                            byteBuffer[offset + 2:offset + 4], "little", signed=True
                        )
                        vitalsign["RangeProfile"].append(
                            pow(real * real + imag * imag, 0.5)
                        )

            # Always advance by the declared payload length. This is required
            # when standard OOB TLVs precede the custom vital-signs TLV.
            idX = payload_end

        byteBuffer[:byteBufferLength - totalPacketLen] = byteBuffer[totalPacketLen:byteBufferLength]
        byteBuffer[byteBufferLength - totalPacketLen:] = 0
        byteBufferLength -= totalPacketLen

    return dataOK, frameNumber, vitalsign


def update():
    global vitalsign, s1, s2, s3, s4, s5, s6
    global Breathsignal, Heartbeatsignal, Chestdisplacement
    global Rangeprofile, Breathenerge, Heartenerge

    dataOk, frameNumber, vitalsign = readAndParseData68xx(Dataport, configParameters)
    if dataOk:
        Breathsignal.append(vitalsign["outputFilterBreathOut"])
        Heartbeatsignal.append(vitalsign["outputFilterHeartOut"])
        Chestdisplacement.append(float(vitalsign["unwrapPhasePeak_mm"]))
        Breathenerge.append(float(vitalsign["sumEnergyBreathWfm"]) / 1e6)
        Heartenerge.append(float(vitalsign["sumEnergyHeartWfm"]))

        if len(Breathsignal) > 250: Breathsignal.pop(0)
        if len(Heartbeatsignal) > 250: Heartbeatsignal.pop(0)
        if len(Chestdisplacement) > 250: Chestdisplacement.pop(0)
        if len(Breathenerge) > 250: Breathenerge.pop(0)
        if len(Heartenerge) > 250: Heartenerge.pop(0)

        if vitalsign.__contains__("RangeProfile"):
            Rangeprofile = vitalsign["RangeProfile"]

        s1.setData(np.array((list(range(0, 250)), Breathsignal)).T)
        s2.setData(np.array((list(range(0, 250)), Heartbeatsignal)).T)
        s3.setData(np.array((list(range(0, 250)), Chestdisplacement)).T)
        if Rangeprofile:
            range_axis = np.arange(len(Rangeprofile)) * configParameters["rangeResolutionMeters"]
            s4.setData(range_axis, Rangeprofile)
        s5.setData(np.array((list(range(0, 250)), Breathenerge)).T)
        s6.setData(np.array((list(range(0, 250)), Heartenerge)).T)

        labelItem1.setText(
            text=f'Breath Rate: {vitalsign["breathingRateEst_FFT"]:.1f} BPM',
            size='12pt',
        )
        labelItem2.setText(
            text=f'Heart Rate: {vitalsign["heartRateEst_FFT"]:.1f} BPM',
            size='12pt',
        )

        QtWidgets.QApplication.processEvents()

    return dataOk


# ======== serial ========
CLIport, Dataport = serialConfig(configFileName)
configParameters = parseConfigFile(configFileName)

# ===== GUI =====
app = QtWidgets.QApplication([])

pg.setConfigOption('background', 'w')
win = pg.GraphicsLayoutWidget(title="Vital Sign")
win.resize(1200, 700)

p1 = win.addPlot(row=1, col=0)
p1.setTitle("Breathing Waveform", color='#008080', size='12pt')
p1.setXRange(0, 250)
p1.setYRange(-2, 2)
p1.setLabel('left', text='Position (mm)')
p1.setLabel('bottom', text='Time (50ms)')
PEN = pg.mkPen(width=2, color='r')
s1 = p1.plot([], [], pen=PEN)
labelItem1 = pg.LabelItem(text='Breath Rate:')
win.addItem(labelItem1, row=0, col=0)

p2 = win.addPlot(row=1, col=1)
p2.setTitle("Cardiac Waveform", color='#008080', size='12pt')
p2.setXRange(0, 250)
p2.setYRange(-2, 2)
p2.setLabel('left', text='Position (mm)')
p2.setLabel('bottom', text='Time (50ms)')
s2 = p2.plot([], [], pen=PEN)
labelItem2 = pg.LabelItem(text='Heartbeat Rate:')
win.addItem(labelItem2, row=0, col=1)

p3 = win.addPlot(row=2, col=0)
p3.setTitle("Chest Displacement")
p3.setXRange(0, 250)
p3.setLabel('left', text='Displacement (a.u.)')
p3.setLabel('bottom', text='Frame')
s3 = p3.plot([], [], pen=PEN)

p4 = win.addPlot(row=2, col=1)
p4.setTitle("Range Profile")
p4.setLabel('left', text='Magnitude')
p4.setLabel('bottom', text='Range (m)')
s4 = p4.plot([], [], pen=PEN)

p5 = win.addPlot(row=3, col=0)
p5.setTitle("Breath Energy")
p5.setXRange(0, 250)
p5.setLabel('left', text='Energy (10^6)')
p5.setLabel('bottom', text='Time')
s5 = p5.plot([], [], pen=PEN)

p6 = win.addPlot(row=3, col=1)
p6.setTitle("Cardiac Energy")
p6.setXRange(0, 250)
p6.setLabel('left', text='Energy')
p6.setLabel('bottom', text='Time')
s6 = p6.plot([], [], pen=PEN)

labelItem3 = pg.LabelItem(text='Range Start:')
win.addItem(labelItem3, row=5, col=0)
labelItem3.setText(text='Range Start: ' + str(configParameters["rangeStart"]) + " m")

labelItem4 = pg.LabelItem(text='Range End:')
win.addItem(labelItem4, row=5, col=1)
labelItem4.setText(text='Range End: ' + str(configParameters["rangeEnd"]) + " m")

labelItem5 = pg.LabelItem(text='Max Range:')
win.addItem(labelItem5, row=6, col=0)
labelItem5.setText(text='Max Range: ' + str(configParameters["maxRange"]) + " m")

labelItem6 = pg.LabelItem(text='Resolution:')
win.addItem(labelItem6, row=6, col=1)
labelItem6.setText(text='Resolution: ' + str(configParameters["rangeResolutionMeters"]) + " m")


# ===== Timer refresh =====
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)


# ===== Show window =====
win.show()
sys.exit(app.exec_())
