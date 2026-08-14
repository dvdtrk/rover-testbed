#!/usr/bin/env python3
"""
Dual-Pico rover control script.

REQUIRED: pyserial

install with:
pip install pyserial --break-system-packages

Reads WASD from the keyboard (single keypress, no Enter needed) and sends
the appropriate single-char command to the LEFT and RIGHT Pico serial ports
simultaneously, implementing differential-drive style steering:

	w = forward   (both sides forward)
	s = backward  (both sides backward)
	a = turn left  (left side stops/reverses, right side forward -> pivot left)
	d = turn right (right side stops/reverses, left side forward -> pivot right)
	x = stop both sides
	q = quit

Also prints telemetry (current/encoder lines) from both boards, prefixed
with [LEFT] / [RIGHT], as they arrive.

Requires firmware on both Picos that understands single-char serial commands:
	'f' = forward half speed
	'b' = backward half speed
	's' = stop
(see the accompanying updated motor_controller.c standalone test)
"""

import serial
import threading
import sys
import termios
import tty
import time

LEFT_PORT = "/dev/ttyAMA2"
RIGHT_PORT = "/dev/ttyAMA3"
BAUD = 115200

TURN_PIVOT_MODE = "stop"  # "stop" = pivot side stops, "reverse" = pivot side reverses (tighter turn)

# Control bytes -- must match CMD_FORWARD / CMD_BACKWARD / CMD_STOP in the firmware.
# Deliberately non-printable so they can never appear inside the firmware's own
# printf telemetry text (see wasd_main_hardened.c for why this matters).
CMD_FORWARD = b"\x01"
CMD_BACKWARD = b"\x02"
CMD_STOP = b"\x03"

# Firmware auto-stops if a move command isn't refreshed within CMD_WATCHDOG_MS (500ms).
# Resend the current command at this interval while a direction key is held.
RESEND_INTERVAL_S = 0.15

# One side's motor is physically mounted as a mirror image of the other (common on
# rovers -- motors face outward). Sending the SAME raw command to both sides makes
# them spin in opposite real-world directions. INVERT_RIGHT flips the right side's
# commands so "logical forward" drives both wheels the same real-world direction.
#
# If pressing 'w' spins in a circle instead of driving straight, flip this value.
INVERT_RIGHT = True


def send_direction(ser, invert, want_forward):
	"""Send whichever raw command makes this side move in the requested
	logical direction, accounting for mirrored motor mounting."""
	if want_forward:
		cmd = CMD_BACKWARD if invert else CMD_FORWARD
	else:
		cmd = CMD_FORWARD if invert else CMD_BACKWARD
	send_cmd(ser, cmd)


def stop_side(ser):
	send_cmd(ser, CMD_STOP)


def open_port(path):
	try:
		return serial.Serial(path, BAUD, timeout=0.1)
	except serial.SerialException as e:
		print(f"Failed to open {path}: {e}")
		sys.exit(1)

print_lock = threading.Lock() # to prevent left and right switching places in terminal
def reader_thread(ser, label, stop_event):
	"""Continuously read lines from a serial port and print them labeled."""
	buf = b""
	while not stop_event.is_set():
		try:
			data = ser.read(256)
			if data:
				buf += data
				while b"\n" in buf:
					line, buf = buf.split(b"\n", 1)
					text = line.decode(errors="replace").strip()
					if text:
						with print_lock:
							print(f"[{label}] {text}")
		except serial.SerialException:
			break


def send_cmd(ser, cmd_byte):
	try:
		ser.write(cmd_byte)
	except serial.SerialException as e:
		print(f"Write failed: {e}")


def get_key():
	"""Read a single keypress from stdin without waiting for Enter.
	Uses cbreak mode (not raw mode) so output newline processing (\\n -> \\r\\n)
	stays intact. Raw mode disables that and causes staggered/staircase output
	from the reader threads' print() calls."""
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setcbreak(fd)
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch

def main():
	print(f"Opening LEFT on {LEFT_PORT} and RIGHT on {RIGHT_PORT} at {BAUD} baud...")
	left = open_port(LEFT_PORT)
	right = open_port(RIGHT_PORT)

	stop_event = threading.Event()
	t_left = threading.Thread(target=reader_thread, args=(left, "LEFT", stop_event), daemon=True)
	t_right = threading.Thread(target=reader_thread, args=(right, "RIGHT", stop_event), daemon=True)
	t_left.start()
	t_right.start()

	print("Connected. Controls: w=forward  s=backward  a=turn left  d=turn right  x=stop  q=quit")
	print("(single keypress, no Enter needed)\n")

	try:
		while True:
			key = get_key().lower()

			if key == "q":
				print("\nQuitting...")
				break

			elif key == "w":
				# logical forward on both sides -- drives straight
				send_direction(left, False, True)
				send_direction(right, INVERT_RIGHT, True)
				print(">> FORWARD")

			elif key == "s":
				# logical backward on both sides -- drives straight
				send_direction(left, False, False)
				send_direction(right, INVERT_RIGHT, False)
				print(">> BACKWARD")

			elif key == "a":
				# pivot left in place: left side backward, right side forward
				send_direction(left, False, False)
				send_direction(right, INVERT_RIGHT, True)
				print(">> TURN LEFT (pivot)")

			elif key == "d":
				# pivot right in place: left side forward, right side backward
				send_direction(left, False, True)
				send_direction(right, INVERT_RIGHT, False)
				print(">> TURN RIGHT (pivot)")

			elif key == "x":
				send_cmd(left, CMD_STOP)
				send_cmd(right, CMD_STOP)
				print(">> STOP")

			elif key == "\x03":  # Ctrl-C
				print("\nQuitting...")
				break

	finally:
		# Safety: always stop motors on exit
		send_cmd(left, CMD_STOP)
		send_cmd(right, CMD_STOP)
		time.sleep(0.1)
		stop_event.set()
		left.close()
		right.close()
		print("Motors stopped, ports closed.")


if __name__ == "__main__":
	main()
