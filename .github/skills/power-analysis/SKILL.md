---
name: power-analysis
description: Skill for analyzing power consumption data and generating insights for energy efficiency. Use when analyzing power data and generating insights for energy efficiency.
---

# Power Analysis

This skill helps you analyze power consumption data and generate insights for energy efficiency. It is designed to work with the Power Draw Analyzer agent, which specializes in analyzing power consumption data and generating insights for energy efficiency.

## Process

1. Use the convert_log_to_list() tool to convert the latest .wpilog file into a readable format.
2. Use a large read window policy by default:
   - Read 4500 to 5000 lines per call when exploring large files.
   - Continue with sequential chunks automatically until the entire relevant file is covered.
   - Keep a running summary across chunks.
3. Look at relevant information for robot specific details.
4. If a read is too large for context limits, retry with smaller chunks (for example 2000 to 2500 lines) and continue.
5. Generate insights and recommendations for improving energy efficiency based on the analysis.
6. Present findings and recommendations clearly and concisely.

## Tool Output Format

The tool `convert_log_to_list()` converts the latest .wpilog file to a readable markdown output with each timestamp using the following format:

  timestamp_ms: {
  entry_name: [avg, min, max, std_dev]
  ...}

The output will be written to a file called ConvertedLog.md at .github/tools/ConvertedLog.md. If the file already exists, it will be overwritten which is ok to do.

Each entry will contain the average, minimum, maximum, and standard deviation of the values between the previous and current timestamp. The entry name is formatted as deviceId_dataType, where deviceId is the device ID of the entry and dataType is the type of data being logged (e.g. SupplyVoltage, SupplyCurrent, etc.) but shorter (e.g. "SupVol" instead of "SupCur"). The timestamp is in milliseconds since the start of the log file.

## Relevant Information

Look at RobotCharacteristics.md for relevant information about the robot's power usage and characteristics. This file contains details about the robot specifics such as current limits and CAN Id numbers. It also contains information about the robot such as irrelevant details in the log file and other needed information such as the brownout voltage.