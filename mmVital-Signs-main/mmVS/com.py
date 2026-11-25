import os
import time


CLI_PROMPT = "mmwDemo:/>"


def _read_cli_response(cli_port, command, timeout_s):
    """Wait for the radar CLI prompt and return the complete command response."""
    deadline = time.monotonic() + timeout_s
    response = bytearray()

    while time.monotonic() < deadline:
        waiting = cli_port.in_waiting
        if waiting:
            response.extend(cli_port.read(waiting))
            text = response.decode(errors="replace")
            if CLI_PROMPT in text:
                return text
        else:
            time.sleep(0.01)

    text = response.decode(errors="replace")
    raise RuntimeError(
        f"CLI timeout after command '{command}'. Radar response:\n{text or '<none>'}"
    )

def serialConfig(configFileName, cli_port=None, data_port=None):
    import serial

    global CLIport
    global Dataport

    cli_port = cli_port or os.environ.get("MMWAVE_CLI_PORT", "COM10")
    data_port = data_port or os.environ.get("MMWAVE_DATA_PORT", "COM9")
    CLIport = serial.Serial(cli_port, 115200, timeout=0.1, write_timeout=1.0)
    Dataport = serial.Serial(data_port, 921600, timeout=0)

    # Discard only the banner/prompt that may already be pending from boot.
    time.sleep(0.2)
    if CLIport.in_waiting:
        startup_text = CLIport.read(CLIport.in_waiting).decode(errors="replace")
        if startup_text.strip():
            print(startup_text, end="" if startup_text.endswith("\n") else "\n")

    with open(configFileName, encoding="utf-8") as config_file:
        config = [line.strip() for line in config_file]
    for command in config:
        if not command or command.startswith("%"):
            continue
        print(f">>> {command}")
        CLIport.write((command + '\n').encode())
        CLIport.flush()
        response = _read_cli_response(
            CLIport,
            command,
            timeout_s=10.0 if command.startswith("sensorStart") else 3.0,
        )
        print(response, end="" if response.endswith("\n") else "\n")
        if "Error" in response or "not recognized" in response:
            raise RuntimeError(
                f"Radar rejected CLI command '{command}'. Response:\n{response}"
            )

    time.sleep(0.2)
    pending_data = Dataport.in_waiting
    if pending_data:
        print(f"Data UART is active: {pending_data} byte(s) waiting.")
    else:
        print(
            "WARNING: sensor configuration completed, but Data UART has "
            "no bytes after 0.2 s."
        )

    return CLIport, Dataport


def parseConfigFile(configFileName):
    configParameters = {
        # The AWR1843 firmware currently fixes these in vital_signs.c instead
        # of exposing the legacy vitalSignsCfg CLI command.
        "rangeStart": 0.30,
        "rangeEnd": 0.90,
    }
    with open(configFileName, encoding="utf-8") as config_file:
        config = [line.strip() for line in config_file]
    for i in config:
        if not i or i.startswith("%"):
            continue
        splitWords = i.split()
        numRxAnt = 4
        numTxAnt = 2
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

            digOutSampleRate = int(splitWords[11])
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

        elif "vitalSignsCfg" in splitWords[0]:
            rangeStart = float(splitWords[1])
            rangeEnd = float(splitWords[2])
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (
            2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    configParameters["rangeStart"] = locals().get(
        "rangeStart", configParameters["rangeStart"]
    )
    configParameters["rangeEnd"] = locals().get(
        "rangeEnd", configParameters["rangeEnd"]
    )
    return configParameters
