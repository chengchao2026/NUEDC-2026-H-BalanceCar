import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser(
        description="Repeatedly send $B,<valid>,<x> frames to test the STM32 single-axis balance loop.")
    parser.add_argument("port", help="Serial port wired to STM32 PA3, e.g. COM5")
    parser.add_argument("x", type=int, nargs="?", default=160,
                        help="Simulated ball x coordinate (default: 160, i.e. center)")
    parser.add_argument("--valid", type=int, choices=(0, 1), default=1,
                        help="1=ball detected, 0=no ball (forces immediate stop)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=0.05,
                        help="Seconds between frames, default 0.05 (50 ms)")
    args = parser.parse_args()

    frame = "$B,{},{}\n".format(args.valid, args.x).encode("ascii")
    ser = serial.Serial(args.port, args.baud, timeout=0)

    print("Sending {!r} every {:.0f} ms to {} - Ctrl+C to stop".format(
        frame, args.interval * 1000, args.port))
    print("Stopping this script should make the motor auto-stop within ~150 ms.")

    try:
        while True:
            ser.write(frame)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
