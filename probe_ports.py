import serial.tools.list_ports
import serial

def probe_ports():
    ports = serial.tools.list_ports.comports()
    print(f"Found {len(ports)} ports:")
    for port in ports:
        print(f"Port: {port.device}, Description: {port.description}")
        try:
            s = serial.Serial(port.device)
            print(f"  Successfully opened {port.device}")
            s.close()
        except Exception as e:
            print(f"  Failed to open {port.device}: {e}")

if __name__ == "__main__":
    probe_ports()
