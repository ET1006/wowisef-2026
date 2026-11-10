import pygame
import serial
import time
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Keyboard Bridge Controller")
    parser.add_argument("--port", type=str, required=True, help="Serial port (e.g. /dev/cu.usbmodem101)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"Opening Serial Connection to {args.port}...")
    try:
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.timeout = 0.02
        ser.dtr = False
        ser.rts = False
        ser.open()

        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connected! Streaming active...")
    except Exception as e:
        print(f"Failed to open port: {e}")
        sys.exit(1)

    pygame.init()
    screen = pygame.display.set_mode((400, 200))
    pygame.display.set_caption("ESP32 CLASP Controller")
    clock = pygame.time.Clock()

    running = True
    last_cmd = "NONE"
    last_drive_state = (0, "FORWARD")

    print("\n[READY] Click inside the Pygame window to capture keyboard input.")
    print("Drive: W / S / Down | Controls: U, I, J, K, O, P, Left, Right | E-Stop: Space")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        keys = pygame.key.get_pressed()

        # Drive Motor Controls
        if keys[pygame.K_w]:
            speed, direction = 255, "FORWARD"
        elif keys[pygame.K_s]:
            speed, direction = 120, "FORWARD"
        elif keys[pygame.K_DOWN]:
            speed, direction = 150, "BACKWARD"
        else:
            speed, direction = 0, "FORWARD"

        # Linkage & Servo Commands
        if keys[pygame.K_SPACE]:
            command = "estop"
        elif keys[pygame.K_u]:
            command = "up_attach"
        elif keys[pygame.K_i]:
            command = "up_detach"
        elif keys[pygame.K_j]:
            command = "down_detach"
        elif keys[pygame.K_k]:
            command = "down_attach"
        elif keys[pygame.K_o]:
            command = "both_attach"
        elif keys[pygame.K_p]:
            command = "both_detach"
        elif keys[pygame.K_LEFT]:
            command = "axon_open"
        elif keys[pygame.K_RIGHT]:
            command = "axon_close"
        else:
            command = "NONE"

        mode = "manual"
        payload = f"{speed},{direction},{command},{mode}\n"

        try:
            ser.write(payload.encode('utf-8'))
            ser.flush()

            # Terminal log feedback for command keys
            if command != "NONE" and command != last_cmd:
                print(f"[SENT CMD] -> {command}")
                last_cmd = command
            elif command == "NONE":
                last_cmd = "NONE"

            # Terminal log feedback for motor drive keys
            current_drive_state = (speed, direction)
            if current_drive_state != last_drive_state:
                if speed > 0:
                    print(f"[SENT DRIVE] -> {speed}, {direction}")
                last_drive_state = current_drive_state

        except Exception as e:
            print(f"Transmission error: {e}")
            break

        try:
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line and line != "STS:ONLINE":
                    print(f"[ROBOT REPLY] {line}")
        except Exception as e:
            print(f"Read error: {e}")
            break

        clock.tick(25)

    ser.close()
    pygame.quit()
    print("Bridge closed.")

if __name__ == "__main__":
    main()