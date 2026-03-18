"""
Dynamixel Motor Scanner
Scans all common baud rates and IDs 0-10 to find your XM430-W350-T.
Run this before anything else to confirm comm is working.
"""
import dynamixel_sdk as dxl

PORT          = '/dev/ttyUSB0'
PROTOCOL      = 2.0
SCAN_IDS      = range(0, 11)          # IDs 0–10 (broaden to range(0,253) if needed)
ADDR_MODEL_NO = 0                     # 2-byte model number, always readable
BAUD_RATES    = [57600, 1000000, 115200, 2000000, 3000000, 4000000]

port_handler   = dxl.PortHandler(PORT)
packet_handler = dxl.PacketHandler(PROTOCOL)

if not port_handler.openPort():
    print("[ERROR] Cannot open port. Check USB connection.")
    exit()

print(f"Scanning {PORT} ...\n{'─'*45}")

found = False
for baud in BAUD_RATES:
    port_handler.setBaudRate(baud)
    for dxl_id in SCAN_IDS:
        model, result, error = packet_handler.read2ByteTxRx(
            port_handler, dxl_id, ADDR_MODEL_NO
        )
        if result == dxl.COMM_SUCCESS:
            print(f"✓ FOUND  ID={dxl_id}  Baud={baud}  Model={model}")
            found = True

if not found:
    print("✗ No motor found. Check power supply and cable connections.")

port_handler.closePort()
