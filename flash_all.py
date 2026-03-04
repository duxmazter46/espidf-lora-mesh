import serial.tools.list_ports
import subprocess
import threading
import sys

ESPTOOL = r"C:\Espressif\tools\python\v5.5.2\venv\Scripts\python.exe"
ESPTOOL_SCRIPT = r"C:\esp\v5.5.2\esp-idf\components\esptool_py\esptool\esptool.py"

BOOTLOADER = "build/bootloader/bootloader.bin"
APP = "build/s3_project.bin"
PARTITION = "build/partition_table/partition-table.bin"

BAUD = "460800"

results = {}
lock = threading.Lock()


def build_command(port):
    return [
        ESPTOOL,
        ESPTOOL_SCRIPT,
        "-p", port,
        "-b", BAUD,
        "--before", "default_reset",
        "--after", "hard_reset",
        "--chip", "esp32s3",
        "write_flash",
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "detect",
        "0x0", BOOTLOADER,
        "0x10000", APP,
        "0x8000", PARTITION
    ]


def find_ports():
    ports = serial.tools.list_ports.comports()
    esp_ports = []

    for port in ports:
        if "USB" in port.description or "CP210" in port.description:
            esp_ports.append(port.device)

    return esp_ports


def flash_silent(port):
    print(f"\n========== Flashing {port} ==========")

    cmd = build_command(port)

    process = subprocess.run(cmd, capture_output=True, text=True)

    success = process.returncode == 0

    with lock:
        results[port] = success

    if success:
        print(f"✅ {port} SUCCESS")
    else:
        print(f"❌ {port} FAILED")


def flash_verbose_retry(port):
    print("\n========================================")
    print(f"RETRYING {port}")
    print("Put this board into BOOTLOADER mode now.")
    print("Hold BOOT, press RESET, release RESET, then release BOOT.")
    input("Press ENTER when ready...")

    cmd = build_command(port)

    process = subprocess.run(cmd)  # no capture_output → live log

    success = process.returncode == 0

    if success:
        print(f"✅ {port} SUCCESS on retry")
    else:
        print(f"❌ {port} STILL FAILED")

    return success


def main():
    ports = find_ports()

    if not ports:
        print("No ESP32 found.")
        return

    print("Found ESP32 ports:", ports)

    threads = []

    # First pass (parallel, silent)
    for port in ports:
        t = threading.Thread(target=flash_silent, args=(port,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    # Summary
    print("\n========================")
    print("FLASH SUMMARY (PASS 1)")
    print("========================")

    failed_ports = []

    for port, status in results.items():
        if status:
            print(f"{port} → SUCCESS")
        else:
            print(f"{port} → FAILED")
            failed_ports.append(port)

    print("========================")

    # Retry failed ports
    for port in failed_ports:
        success = flash_verbose_retry(port)
        results[port] = success

    # Final summary
    print("\n========================")
    print("FINAL FLASH SUMMARY")
    print("========================")

    for port, status in results.items():
        if status:
            print(f"{port} → SUCCESS")
        else:
            print(f"{port} → FAILED")

    print("========================")


if __name__ == "__main__":
    main()