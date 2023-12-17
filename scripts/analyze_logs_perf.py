#!/usr/bin/env python3
"""
Analyze log files

# TODO: Individual analyses:
#        - App run time
#        - Time to main
#        - Main duration
#        - Top 10 PerfLogs and include how much cumulative each consumes
"""
import argparse
import os
import re
from collections import defaultdict
from dataclasses import dataclass
from typing import List


@dataclass
class PerfLog():
    log: str
    time_delta: int
    line_num: int

    def __repr__(self) -> str:
        return f"{str(self.time_delta)+'msec':<10} {self.line_num:<5} {self.log}"


class PerfLogAnalysis():
    def __init__(self, perf_logs: List[PerfLog]) -> None:
        self.perf_logs = perf_logs
        self.num_logs = len(perf_logs)
        self.min_delta = min(log.time_delta for log in perf_logs)
        self.max_delta = max(log.time_delta for log in perf_logs)
        self.ave_delta = sum(log.time_delta for log in perf_logs) / len(perf_logs)

    def __repr__(self) -> str:
        generic_log_text = re.split(r'(?<=:)\s*(\d+)', self.perf_logs[0].log, maxsplit=1)[0]
        return (
            f"Log - {generic_log_text}\n"
            f"Num - {self.num_logs}\n"
            f"Min - {self.min_delta}\n"
            f"Max - {self.max_delta}\n"
            f"Ave - {self.ave_delta}\n"
        )


def analyze_logs(perf_logs: List) -> List:
    # Sort perf logs by log message
    perf_logs_per_type = defaultdict(list)
    for log in perf_logs:
        log_generic = re.split(r"(?<=:)\s*(\d+)", log.log, maxsplit=1)[0]
        perf_logs_per_type[log_generic].append(log)

    analysis = []
    for p_logs in perf_logs_per_type.values():
        analysis.append(PerfLogAnalysis(p_logs))

    return analysis


def print_perf_logs(perf: List) -> None:
    for log in perf:
        print(log, end="")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", required=True, help="Log file to analyze")
    parser.add_argument("-p", "--print", help="Print results to console")
    args = parser.parse_args()

    log_file_path = os.path.abspath(args.file)
    perf_logs_all = []

    # Parse log file and create perf log entries
    with open(log_file_path, "r", encoding="utf-8") as f:
        curr_log = None
        prev_log = None
        for line_num, line in enumerate(f.readlines()):
            if not line or line == "\n" or not line[0].isdigit():
                # prev_log = None
                continue

            if not prev_log:
                prev_log = line
                continue

            curr_log = line
            curr_timestamp = int(curr_log.split(" ", maxsplit=1)[0])
            prev_timestamp = int(prev_log.split(" ", maxsplit=1)[0])
            time_delta = curr_timestamp - prev_timestamp
            prev_log_text = prev_log.split(' ', maxsplit=1)[1].strip()
            perf_log = PerfLog(prev_log_text, time_delta, line_num)
            perf_logs_all.append(perf_log)
            prev_log = curr_log

    # Sort all perf logs by performance
    perf_logs_all.sort(key=lambda log: log.time_delta, reverse=True)

    # Analyze logs
    perf_logs_analysis = analyze_logs(perf_logs_all)
    perf_logs_analysis.sort(key=lambda analysis: analysis.ave_delta, reverse=True)

    # Create result file with all PerfLogs
    result_file_all_logs = os.path.splitext(os.path.basename(log_file_path))[0] + "_PERF_LOGS_ALL.txt"
    result_file_all_logs = os.path.join(os.path.dirname(log_file_path), result_file_all_logs)
    with open(result_file_all_logs, "w", encoding="utf-8") as f:
        for log in perf_logs_all:
            f.write(str(log) + "\n")

    # Create analysis file
    result_file_analysis = os.path.splitext(os.path.basename(log_file_path))[0] + "_PERF_ANALYSIS.txt"
    result_file_analysis = os.path.join(os.path.dirname(log_file_path), result_file_analysis)
    with open(result_file_analysis, "w", encoding="utf-8") as f:
        for analysis in perf_logs_analysis:
            f.write(str(analysis) + "\n")

    if args.print:
        print_perf_logs(perf_logs_all)


if __name__ == "__main__":
    main()
