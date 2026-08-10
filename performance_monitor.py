import csv
import re
import sys
import time
import psutil

TARGET_PATTERN = r"LIO_GF_nodelet_manager"
OUTPUT_FILE = "process_usage.csv"
FREQUENCY_HZ = 20
INTERVAL = 1.0 / FREQUENCY_HZ  # same interval as lio pose callback


def get_target_processes(pattern_str):
    pattern = re.compile(pattern_str)
    procs = []
    for proc in psutil.process_iter(["pid", "name"]):
        try:
            cmdline_str = " ".join(proc.cmdline())
            if pattern.search(cmdline_str):
                procs.append(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return procs


def main():
    procs = get_target_processes(TARGET_PATTERN)
    if not procs:
        print(f"No running process found matching pattern '{TARGET_PATTERN}'")
        sys.exit(1)

    pids = [p.pid for p in procs]
    print(
        f"Monitoring process (PIDs: {pids}) at {FREQUENCY_HZ} Hz. Press Ctrl+C to stop."
    )

    for p in procs:
        try:
            p.cpu_percent(interval=None)
        except psutil.NoSuchProcess:
            pass

    with open(OUTPUT_FILE, mode="w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "timestamp",
                "pid",
                "cpu_percent",
                "ram_rss_mb",
                "ram_vms_mb",
                "ram_percent",
            ]
        )

        try:
            while True:
                start_time = time.monotonic()
                timestamp = time.time()

                for p in list(procs):
                    try:
                        cpu = p.cpu_percent(interval=None)
                        mem_info = p.memory_info()
                        mem_percent = p.memory_percent()

                        rss_mb = mem_info.rss / (1024 * 1024)
                        vms_mb = mem_info.vms / (1024 * 1024)

                        writer.writerow(
                            [
                                f"{timestamp:.3f}",
                                p.pid,
                                f"{cpu:.2f}",
                                f"{rss_mb:.2f}",
                                f"{vms_mb:.2f}",
                                f"{mem_percent:.2f}",
                            ]
                        )
                    except psutil.NoSuchProcess:
                        print(f"Process PID {p.pid} terminated.")
                        procs.remove(p)

                csvfile.flush()

                if not procs:
                    print("All target processes exited. Stopping logging.")
                    break

                elapsed = time.monotonic() - start_time
                sleep_duration = max(0.0, INTERVAL - elapsed)
                time.sleep(sleep_duration)

        except KeyboardInterrupt:
            print("\nLogging stopped.")


if __name__ == "__main__":
    main()