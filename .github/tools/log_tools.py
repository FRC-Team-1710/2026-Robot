from fastmcp import FastMCP

# from deepseek_tokenizer import ds_token
from wpiutil.log import DataLogReader
from pathlib import Path

mcp = FastMCP("Log Python Tools")

TARGET_KEYS = ["/SupplyVoltage", "/SupplyCurrent"] # , "/MotorVoltage", "/StatorCurrent"]

def write_string_to_md(filename: str, content: str) -> None:
    script_dir = Path(__file__).parent.resolve()
    file_path = script_dir / filename
    with open(file_path, "w", encoding="utf-8") as file:
        file.write(content)

@mcp.tool()
def convert_log_to_list() -> str:
    """
    Convert the log file to a readable output with each timestamp using the following format:
        timestamp_ms: {
        entry_name: [avg, min, max, std_dev]
        ...}
    Each entry will contain the average, minimum, maximum, and standard deviation of the values between the previous and current timestamp. The entry name is formatted as deviceId_dataType, where deviceId is the device ID of the entry and dataType is the type of data being logged (e.g. SupplyVoltage, SupplyCurrent, etc.) but shorter (e.g. "SupVol" instead of "SupCur"). The timestamp is in milliseconds since the start of the log file.
    The output will be written to a file called ConvertedLog.md at .github/tools/ConvertedLog.md. If the file already exists, it will be overwritten which is ok to do.
    """

    robot_dir = Path(__file__).resolve().parent.parent.parent
    log_file = list(robot_dir.glob("*.wpilog"))
    if not log_file:
        raise FileNotFoundError("No .wpilog file found.")
    reader = DataLogReader(str(log_file[0]))

    entries = {}
    output = {}

    enabled = False
    hasEnabled = False
    disableTimestamp = 0
    matchStart = -1
    currentTimestamp = 0.0

    for record in reader:
        if record.isStart():
            try:
                data = record.getStartData()
            except TypeError:
                continue
            entries[data.entry] = data
            continue

        if record.isControl():
            continue

        entry = entries.get(record.getEntry())
        if entry is None:
            continue

        if "RobotEnable" in entry.name and "Phoenix6" not in entry.name:
            if not enabled and record.getBoolean():
                enabled = True
                hasEnabled = True
                if matchStart == -1:
                    matchStart = record.getTimestamp() / 1000.0
                    currentTimestamp = record.getTimestamp() / 1000.0
            elif enabled and not record.getBoolean():
                enabled = False
                disableTimestamp = (record.getTimestamp() / 1000.0) - matchStart
            continue

        if not hasEnabled:
            continue

        for target_key in TARGET_KEYS:
            if target_key in entry.name:
                break
        else:
            continue

        if record.getTimestamp() / 1000.0 > 500.0 + currentTimestamp: # half second
            currentTimestamp = record.getTimestamp() / 1000.0

        value = None
        try:
            if entry.type == "boolean":
                value = record.getBoolean()
            elif entry.type == "int64":
                value = record.getInteger()
            elif entry.type == "float":
                value = round(record.getFloat(), 1)
            elif entry.type == "double":
                value = round(record.getDouble(), 1)
            elif entry.type in ("string", "json", "msgpack"):
                value = record.getString()
            elif entry.type == "boolean[]":
                value = record.getBooleanArray()
            elif entry.type == "float[]":
                value = record.getFloatArray()
            elif entry.type == "double[]":
                value = record.getDoubleArray()
            elif entry.type == "int64[]":
                value = record.getIntegerArray()
            elif entry.type == "string[]":
                value = record.getStringArray()
            else:
                value = record.getRaw().hex()
        except TypeError:
            value = "invalid"

        if int(currentTimestamp - matchStart) not in output:
            output[int(currentTimestamp - matchStart)] = {}
            
        if entry.name not in output[int(currentTimestamp - matchStart)]:
            output[int(currentTimestamp - matchStart)][entry.name] = []

        output[int(currentTimestamp - matchStart)][entry.name].append(value)

    # Remove all data after last time the robot was enabled for the log as it is unnecessary and adds excessive context
    newOutput = {}
    for key in output:
        if not key > disableTimestamp:
            newOutput[key] = output[key]

    newOutput.pop(max(newOutput.keys()), None)  # Remove the last entry as it is likely incomplete

    newNewOutput = {}
    for key in newOutput:
        newNewOutput[key] = {}
        for dataKey in newOutput[key]:
            avg = round(sum(newOutput[key][dataKey]) / len(newOutput[key][dataKey]), 1)
            maxVal = max(newOutput[key][dataKey])
            minVal = min(newOutput[key][dataKey])
            stdDev = round((sum((x - avg) ** 2 for x in newOutput[key][dataKey]) / len(newOutput[key][dataKey])) ** 0.5, 2)
            newNewOutput[key][dataKey.replace("Phoenix6/TalonFX-", "").replace("StatorCurrent", "StatCur").replace("SupplyCurrent", "SupCur").replace("SupplyVoltage", "SupVol").replace("MotorVoltage", "Vol").replace("/", "_")] = [avg, minVal, maxVal, stdDev]

    write_string_to_md("ConvertedLog.md", str(newNewOutput).replace("'", "").replace("-0.0", "0.0").replace("], ", "]\n").replace("}, ", "}\n").replace(": {", ": {\n"))
    
    return "Converted log file written to ConvertedLog.md"

if __name__ == "__main__":
    mcp.run()
