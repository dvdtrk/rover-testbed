"""
based on https://mackinnon.info/2025/04/20/bare-metal-flashing-of-the-RP2040.html

pico_flash.py:

	GPIO setup (lgpio)
	write_bit() / read_bit()             <- bit level
	swd_write() / swd_read()             <- transaction level
	write_dp/ap() / read_dp/ap()         <- convenience wrappers
	write_word() / read_word()         <- memory access
	halt_cpu() / reset_into_debug()         <- CPU control
	write_core_reg() / read_core_reg()     <- register access
	lookup_rom_function()             <- ROM lookup table
	call_rom_function()             <- ROM function calls
	flash_erase() / flash_program_page()    <- flash operations
	verify_page()                 <- verification
 	flash_binary()                 <- main logic
	cleanup()                 <- GPIO cleanup

Convert .elf to .bin first:
cd ~/micro_ros_raspberrypi_pico_sdk/build
arm-none-eabi-objcopy -O binary pico_micro_ros_example.elf pico_micro_ros_example.bin

Flash:
sudo python3 ~/rover-testbed/pico_flash.py pico_micro_ros_example.bin

"""

DEBUG = True

import os
import sys
import time

os.environ['LG_WD'] = '/tmp'
import lgpio
# http://abyz.me.uk/lg/py_lgpio.html
# help (lgpio)
# help (lgpio.gpio_write)

# dir (lgpio)


SWCLK_PIN = 17 # Pin 11
SWDIO_PIN = 16 # Pin 36

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, SWCLK_PIN, 0)
lgpio.gpio_claim_output(h, SWDIO_PIN, 0)

def set_clk(val):
	lgpio.gpio_write(h, SWCLK_PIN, val)

def set_dio(val):
	lgpio.gpio_write(h, SWDIO_PIN, val)

def get_dio():
	return lgpio.gpio_read(h, SWDIO_PIN)

def release_dio():
	# Drive low first to discharge residual output
	lgpio.gpio_write(h, SWDIO_PIN, 0)
	time.sleep(0.005) # 5ms

	# Switch to input w/ pull down
	lgpio.gpio_claim_input(h, SWDIO_PIN)
	time.sleep(0.005)

def hold_dio():
	lgpio.gpio_claim_output(h, SWDIO_PIN, 0)

def delay():
	time.sleep(0.01) # 1 ms

def write_bit(b):
	set_dio(1 if b else 0) # put bit ovalue on SWDIO
	delay()
	set_clk(1) # target captures data on rising edge (1)
	delay()
	set_clk(0) # ready for next bit

def read_bit():
	delay()
	val = get_dio() # read what pico puts on SWDIO
	set_clk(1) # target captures data on rising egde (1)
	delay()
	set_clk(0) # ready for next bit
	return val

def swd_write(is_ap, addr, data, ignore_ack=False):
	# Sends 32-bit value to a register on the Pico

	if DEBUG:
		print(f"  swd_write: is_ap={is_ap} addr=0x{addr:02X} data=0x{data:08X}")

	# Calculate parity for request
	ones = 0
	if addr & 0b0100:
		ones += 1
	if addr & 0b1000:
		ones += 1
	if is_ap:
		ones += 1

	# Send 8-bit request header
	w_bits = [1, 1 if is_ap else 0, 0,
		1 if (addr & 0b0100) else 0,
		1 if (addr & 0b1000) else 0,
		1 if (ones % 2 == 1) else 0, 0, 1]
	print(f"  write header: {w_bits}")

	write_bit(1)                            # start bit
	write_bit(1 if is_ap else 0)            # 0=DP, 1=AP
	write_bit(0)                            # 0=write
	write_bit(1 if (addr & 0b0100) else 0)  # address bit 2
	write_bit(1 if (addr & 0b1000) else 0)  # address bit 3
	write_bit(1 if (ones % 2 == 1) else 0)  # parity
	write_bit(0)                            # stop bit
	write_bit(1)                            # park bit

	# Release SWDIO so Pico can drive it
	release_dio()
	time.sleep(0.005)
	turnaround = read_bit()  # turnaround bit
	print(f" turnaround: {turnaround}")
	ack_bits=[]
	for _ in range(3):
		bit = read_bit()
		ack_bits.append(bit)

	# Read 3-bit acknowledgement from Pico
	ack = 0
	if ack_bits[0]: ack |= 1
	if ack_bits[1]: ack |= 2
	if ack_bits[2]: ack |= 4

	print(f" write raw ack bits: {ack_bits}")

	# Take SWDIO back
	hold_dio()
	write_bit(0) # turnaround bit

	# Send 32-bit data LSB first
	ones = 0
	for i in range(32):
		bit = (data >> i) & 1
		ones += bit
		write_bit(bit)

	# Send parity
	write_bit(1 if (ones % 2 == 1) else 0)

	if not ignore_ack and ack != 0b001:
			raise Exception(f"SWD write ack failed: {ack}")

def swd_read(is_ap, addr):
# Reads 32 bits

	if DEBUG:
		print(f"  swd_read: is_ap={is_ap} addr=0x{addr:02X}")

	# Calculate parity for request header
	ones = 1  # start with 1 because read bit is always 1
	if addr & 0b0100:
		ones += 1
	if addr & 0b1000:
		ones += 1
	if is_ap:
		ones += 1

	# Send 8-bit request header
	h_bits = [1, 1 if is_ap else 0, 1,
		1 if (addr & 0b0100) else 0,
		1 if (addr & 0b1000) else 0,
		1 if (ones % 2 == 1) else 0, 0, 1]
	print(f"  header: {h_bits}")

	write_bit(1)                             # start bit
	write_bit(1 if is_ap else 0)             # 0=DP, 1=AP
	write_bit(1)                             # 1=read
	write_bit(1 if (addr & 0b0100) else 0)   # address bit 2
	write_bit(1 if (addr & 0b1000) else 0)   # address bit 3
	write_bit(1 if (ones % 2 == 1) else 0)   # parity
	write_bit(0)                             # stop bit
	write_bit(1)                             # park bit

	# Release SWDIO so Pico can drive it
	release_dio()
	time.sleep(0.005) # give time to switch direction
	turnaround = read_bit() # turnaround bit
	print(f" read turnaround: {turnaround}")

	# debug
	ack_bits = []
	for _ in range(3):
		bit = read_bit()
		ack_bits.append(bit)

	print(f" raw ack bits: {ack_bits}")

	# Read 3-bit acknowledgement
	ack = 0
	if ack_bits[0]: ack |= 1
	if ack_bits[1]: ack |= 2
	if ack_bits[2]: ack |= 4

	if ack != 0b001:
		# Still consume the data phase to keep the protocol in sync
		for _ in range(32):
			read_bit()
		read_bit()  # parity
		hold_dio()
		write_bit(0)
		raise Exception(f"SWD read ack failed: {ack}")


	# Read 32-bit data LSB first
	data = 0
	ones = 0
	for i in range(32):
		bit = read_bit()
		ones += bit
		data |= (bit << i)

	# Read parity bit
	parity = read_bit()
	expected_parity = ones % 2
	if parity != expected_parity:
		hold_dio()
		write_bit(0)
		raise Exception(f"SWD read parity error: got {parity}, expected {expected_parity} (data=0x{data:08X})")

	# Take SWDIO back AFTER reading data
	hold_dio()
	write_bit(0) # turnaround bit

	return data

# Some convenience wrappers:
def write_dp(addr, data, ignore_ack=False):
	swd_write(False, addr, data, ignore_ack)

def write_ap(addr, data):
	try:
		swd_write(True, addr, data)
	except Exception:
		clear_sticky_errors()
		time.sleep(0.01)
		swd_write(True, addr, data)

def read_dp(addr):
	return swd_read(False, addr)

def read_ap(addr):
	try:
		return swd_read(True, addr)
	except Exception:
		clear_sticky_errors()
		time.sleep(0.01)
		return swd_read(True, addr)

def clear_sticky_errors():
	time.sleep(0.005)
	ctrl_stat = read_dp(0x4)
	print(f"CTRL/STAT before clear: 0x{ctrl_stat:08X} "
	      f"(STICKYERR={bool(ctrl_stat & (1<<5))}, "
	      f"STICKYORUN={bool(ctrl_stat & (1<<1))}, "
	      f"WDATAERR={bool(ctrl_stat & (1<<7))})")
	write_dp(0x0, 0x1E) # ABORT: clears STKERRCLR, WDERRCLR, ORUNERRCLR
	time.sleep(0.005)
	ctrl_stat_after = read_dp(0x4)
	print(f"CTRL/STAT after clear: 0x{ctrl_stat_after:08X}")

def swd_init():
	# Initializes debug interface on the Pico

	print("Initializing SWD...")
	# Follows 14 steps (Bruce MacKinnon)

	# 1-5: Protocal reset sequence

	# Step 1: Initial state - pull pins low then send 8 ones
	set_clk(0)
	set_dio(0)
	for _ in range(8):
		write_bit(1)

	# Step 2: Send 128-bit selection alert
	selection_alert = [
		0x92, 0xF3, 0x09, 0x62, 0x95, 0x2D, 0x85, 0x86,
		0xE9, 0xAF, 0xDD, 0xE3, 0xA2, 0x0E, 0xBC, 0x19
	]

	for byte in selection_alert:
		for i in range(8):
			write_bit((byte >> i) & 1)

	# Step 3a: Send 4 zeros
	for _ in range(4):
		write_bit(0)

	# Step 3b: Send activation code 0x1a LSB first
	activation = 0x1a
	for i in range(8):
		write_bit((activation >> i) & 1)

	# Step 4: Line reset - send 64 ones
	for _ in range(64):
		write_bit(1)

	# Step 5: Send 8 zeros
	for _ in range(8):
		write_bit(0)

	# Step 6: DP target select, selects which processor core to connect to
	# Write core 0 address
	# ignore_ack=True because target doesn't acknowledge this specific step
	write_dp(0xc, 0x01002927, ignore_ack=True)

	# Immediately read DPIDR to confirm/latch the target selection —
	# per ADI spec B4.3.4, nothing else (especially not another line
	# reset) may happen between the TARGETSEL write and this read.
	idcode = read_dp(0x0)

	print(f"IDCODE: 0x{idcode:08X}")
	if idcode != 0x0BC12477:
		raise Exception(f"Unexpected IDCODE: 0x{idcode:08X}")

	# Diagnostic: confirm we actually landed on core0's real DP, not a fallback
	write_dp(0x8, 0x00000003)   # SELECT: DPBANKSEL = 3, exposes DLPIDR at 0x4
	dlpidr = read_dp(0x4)
	print(f"DLPIDR: 0x{dlpidr:08X} (TINSTANCE={dlpidr >> 28:X})")
	write_dp(0x8, 0x00000000)   # back to bank 0 before continuing normal setup

	# Step 7 no longer exists lol

	# 8-14: Power up and configure debug system

	# 8-14: Power up and configure debug system

	# Step 8: Abort - clear any pending operations
	write_dp(0x0, 0x1E)

	# Step 9: Select AP 0, AP bank 0, SW-DP bank 0
	write_dp(0x8, 0x00000000)

	# Step 10: Power up debug system
	write_dp(0x4, (1<<30) | (1<<28) | (1<<0))

	# Step 11: Verify power up
	ctrl_stat = read_dp(0x4)

	if not (ctrl_stat & (1<<31)) or not (ctrl_stat & (1<<29)):
		raise Exception("Debug power up failed")
	print("Debug power up confirmed")

	# Step 12: Check AP ID
	write_dp(0x8, 0x000000F0)
	read_ap(0xc)
	ap_id = read_dp(0xc)
	print(f"AP ID: 0x{ap_id:08X}")

	# Step 13: Select AP bank 0
	write_dp(0x8, 0x00000000)

	# Step 14: Configure AP transfer mode
	# Auto increment on, word transfer
	write_ap(0x0, CSW_VALUE)

	# Diagnostic if the write actually took effect
	read_ap(0x0)
	csw_readback = read_dp(0xC)
	print(f"CSW readback (wrote 0x{CSW_VALUE:08X}): 0x{csw_readback:08x}")

	print("SWD initialization complete")


def swd_connect(max_attempts=5):
	for attempt in range(1, max_attempts + 1):
		try:
			swd_init()
			return
		except Exception as e:
			print(f"Connect attempt {attempt} failed: {e}")
			time.sleep(0.2)
	raise Exception("Failed to connect to target after multiple attempts. Try setting pin 30 (RUN) on the Pico to GND for a second.")


#Memory read/write

def write_word(addr, data):
	# Write address to TAR (Transfer Address Register)
	# (Sets target address)
	write_ap(0x4, addr)
	# Write data to DRW (Data Read/Write) register
	write_ap(0xC, data)

def read_word(addr):
	# Write address to TAR register
	write_ap(0x4, addr)
	# Initiate read from DRW (data goes to DP buffer)
	read_ap(0xC)
	# Fetch actual result from DP RDBUFF register
	return read_dp(0xC)

	#Quirk of ARM CoreSight Architecture

def read_half_word(addr):
	# Force word alignment by masking bottom 2 bits
	# b/c SWD only does 32-bit reads
	# Mask bottom 2 bits to force address to word boundary
	write_ap(0x4, addr & 0xFFFFFFFC)
	read_ap(0xC)
	word = read_dp(0xC)
	# Return correct 16 bits depending on alignment
	if (addr & 0x3) == 0:
		return word & 0xFFFF          # even half word
	else:
		return (word >> 16) & 0xFFFF  # odd half word

	# Needed for reading ROM function lookup table which uses 16-bit addresses

def halt_cpu():
	# Writes to DHCSR (Debug Halting Control and Status Register)

	print("Halting CPU...")
	# Write magic key + halt bits to DHCSR
	# Magic key = 0xA05F000B (0xA05F) disables interrupts + halt + enable debug
	write_word(0xE000EDF0, 0xA05F000B)
	# Verify halted
	dhcsr = read_word(0xE000EDF0)
	if not (dhcsr & (1 << 17)):
		raise Exception("CPU failed to halt")
	print("CPU halted")
	# Check S_HALT bit (bit 17) to confirm halt success

def resume_cpu():
	write_word(0xE000EDF0, 0xA05F0000)
	print("CPU resumed")

def reset_into_debug():
	print("Resetting into debug mode...")

	write_word(0xE000EDF0, 0xA05F0001)   # DHCSR: C_DEBUGEN = 1
	write_word(0xE000EDFC, 0x00000001)   # DEMCR: VC_CORERESET
	write_word(0xE000ED0C, 0x05FA0004)   # AIRCR: SYSRESETREQ

	for _ in range(200):
		dhcsr = read_word(0xE000EDF0)
		if not (dhcsr & (1 << 25)):
			break
		time.sleep(0.001)
	else:
		raise Exception("Timed out waiting for S_RESET_ST to clear")

	# Re-establish the debug port / AP
	write_dp(0x0, 0x1E)
	write_dp(0x8, 0x00000000)
	write_dp(0x4, (1 << 30) | (1 << 28) | (1 << 0))
	ctrl_stat = read_dp(0x4)
	if not (ctrl_stat & (1 << 31)) or not (ctrl_stat & (1 << 29)):
		raise Exception("Debug power up failed after reset")
	write_dp(0x8, 0x00000000)
	write_ap(0x0, CSW_VALUE) # reconfigure CSW (auto-inc, word)
	clear_sticky_errors()

	# Diagnostic: verify basic memory reads work post-reset before
	# assuming anything about DHCSR specifically
	reset_vector = read_word(0x00000004)
	print(f"Reset vector (sanity check, should be nonzero): 0x{reset_vector:08X}")

	# Force a halt outright, in case C_DEBUGEN/VC_CORERESET got wiped by the reset
	write_word(0xE000EDF0, 0xA05F0003)   # C_DEBUGEN | C_HALT
	time.sleep(0.01)

	dhcsr = read_word(0xE000EDF0)
	print(f"DHCSR after reset: 0x{dhcsr:08X}")
	if not (dhcsr & (1 << 17)):
		raise Exception("CPU failed to halt after reset")
	print("CPU reset and halted")

# Core register numbers
REG_R0 = 0
REG_R1 = 1
REG_R2 = 2
REG_R3 = 3
REG_R7 = 7
REG_SP = 13 # MSP
REG_LR = 14
REG_PC = 15

# Core register read/read to manipulate CPU registers (PC, SP, R0-R12 etc)
# Needed to call ROM functions

def write_core_reg(reg_num, value):
	# Write value to DCRDR
	write_word(0xE000EDF8, value)
	# Write selector to DCRSR (bit 16 = write, bits 6:0 = register number)
	write_word(0xE000EDF4, (1 << 16) | reg_num)
	# Poll DHCSR until S_REGRDY (bit 16) is set
	for _ in range(100):
		dhcsr = read_word(0xE000EDF0)
		if dhcsr & (1 << 16):
			return
	raise Exception(f"Timeout writing core register {reg_num}")

def read_core_reg(reg_num):
	# Write selector to DCRSR (bit 16 = 0 for read)
	write_word(0xE000EDF4, reg_num)
	# Poll DHCSR until S_REGRDY (bit 16) is set
	for _ in range(100):
		dhcsr = read_word(0xE000EDF0)
		if dhcsr & (1 << 16):
			return read_word(0xE000EDF8)
	raise Exception(f"Timeout reading core register {reg_num}")

def call_rom_function(func_addr, r0=0, r1=0, r2=0, r3=0):
	# Find the debug trampoline function in ROM
	# Takes the address from R7, calls that function, hits a BKPT instruction on return, halting the CPU
	trampoline = lookup_rom_function("DT")

	# Set function arguments in R0-R3
	write_core_reg(REG_R0, r0)
	write_core_reg(REG_R1, r1)
	write_core_reg(REG_R2, r2)
	write_core_reg(REG_R3, r3)

	# Set stack pointer to valid RAM location
	write_core_reg(REG_SP, 0x20000080)

	# Put target function address in R7 (trampoline reads this)
	write_core_reg(REG_R7, func_addr | 1)  # |1 for thumb mode

	# Set PC to trampoline
	write_core_reg(REG_PC, trampoline | 1)

	# Clear debug fault status
	dfsr = read_word(0xE000ED30)
	write_word(0xE000ED30, dfsr)

	# Resume CPU — trampoline will call function then halt
	write_word(0xE000EDF0, 0xA05F0009)

	# Wait for halt (BKPT hit at end of trampoline)
	for _ in range(10000):
		dhcsr = read_word(0xE000EDF0)
		if dhcsr & (1 << 17):
			return
		time.sleep(0.0001)

	raise Exception("Timeout waiting for ROM function to complete")

	# ROM FUNCTIONS TO USE:

	# "IF" -> connect_internal_flash() ->  resets QSPI flash
	# "EX" -> flash_exit_xip() —> puts flash in write mode
	# "RE" -> flash_range_erase() —> erases flash
	# "RP" -> flash_range_program() —> writes data to flash
	# "FC" -> flash_flush_cache() —> flushes cache
	# "CX" -> flash_enter_cmd_xip() —> re-enables flash reading

# Flash constants:
FLASH_START = 0x10000000 # XIP flash start address
RAM_WORK_AREA = 0x20000100 # where we stage data in RAM
PAGE_SIZE = 4096 # 4K pages
CSW_VALUE = (1 << 31) | (0b01 << 4) | 0b010   # DbgSwEnable=1, AddrInc=1, Size=word

# Flash operations:

def flash_connect():
	print("Connecting to flash...")
	func = lookup_rom_function("IF")
	call_rom_function(func)

def flash_exit_xip():
	print("Exiting XIP mode...")
	func = lookup_rom_function("EX")
	call_rom_function(func)

def flash_flush_cache():
	func = lookup_rom_function("FC")
	call_rom_function(func)

def flash_enter_xip():
	print("Re-entering XIP mode...")
	func = lookup_rom_function("CX")
	call_rom_function(func)

def flash_erase(addr, size):
	print(f"Erasing flash at 0x{addr:08X} size {size} bytes...")
	func = lookup_rom_function("RE")
	# Arguments: addr (offset from flash start), size, block_size, block_cmd
	call_rom_function(func,
		r0=addr - FLASH_START, # offset from flash start
		r1=size, # size to erase
		r2=4096, # erase block size (4K)
		r3=0x20) # erase command (sector erase)
	print("Erase complete")

def flash_program_page(addr, data):
	# Write 4K of data to RAM work area first
	offset = 0
	for i in range(0, len(data), 4):
	# Pack 4 bytes into a 32-bit word
		word = (data[i]
		| (data[i+1] << 8)
		| (data[i+2] << 16)
		| (data[i+3] << 24))
	write_word(RAM_WORK_AREA + offset, word)
	offset += 4

	# Call ROM flash_range_program to copy from RAM to flash
	func = lookup_rom_function("RP")
	call_rom_function(func,
		r0=addr - FLASH_START, # flash offset
		r1=RAM_WORK_AREA, # source in RAM
		r2=PAGE_SIZE) # size

def verify_page(addr, data):
	# Verify after flashing
	for i in range(0, len(data), 4):
		word = (data[i] |
			(data[i+1] << 8) |
			(data[i+2] << 16) |
			(data[i+3] << 24))
		flash_word = read_word(addr + i)
	if flash_word != word:
		raise Exception(f"Verify failed at 0x{addr+i:08X}: "
			f"expected 0x{word:08X} got 0x{flash_word:08X}")


def call_rom_function_raw(func_addr, r0=0, r1=0, r2=0, r3=0):
	"""Call a ROM function directly, without the DT trampoline.
	Only used to bootstrap table_lookup() before DT's address is known."""
	BKPT_STUB = 0x20000040
	write_word(BKPT_STUB, 0xBE00BE00)   # two BKPT #0 instructions, back to back

	write_core_reg(REG_R0, r0)
	write_core_reg(REG_R1, r1)
	write_core_reg(REG_R2, r2)
	write_core_reg(REG_R3, r3)
	write_core_reg(REG_SP, 0x20000080)
	write_core_reg(REG_LR, BKPT_STUB | 1)   # on return, land on our BKPT stub
	write_core_reg(REG_PC, func_addr | 1)   # jump straight into the function

	dfsr = read_word(0xE000ED30)
	write_word(0xE000ED30, dfsr)
	write_word(0xE000EDF0, 0xA05F0009)      # resume

	for _ in range(10000):
		dhcsr = read_word(0xE000EDF0)
		if dhcsr & (1 << 17):
			return
		time.sleep(0.0001)
	raise Exception("Timeout waiting for ROM function to complete")


def lookup_rom_function(code_str):
	c1 = ord(code_str[0])
	c2 = ord(code_str[1])
	code = c1 | (c2 << 8)

	table_ptr = read_half_word(0x00000014)   # pointer to the function table
	lookup_fn = read_half_word(0x00000018)   # address of table_lookup(), fixed, no lookup needed

	call_rom_function_raw(lookup_fn, r0=table_ptr, r1=code)
	result = read_core_reg(REG_R0)

	if result == 0:
		raise Exception(f"ROM function '{code_str}' not found")
	return result


# Main flash function:
def flash_binary(filename):

	if DEBUG:
		print("Starting in 10 seconds")
		time.sleep(10)

	# Read binary file
	with open(filename, 'rb') as f:
		binary = f.read()

	# Pad to multiple of 4K
	remainder = len(binary) % PAGE_SIZE
	if remainder:
		binary += b'\xff' * (PAGE_SIZE - remainder)

	total_pages = len(binary) // PAGE_SIZE
	print(f"Flashing {len(binary)} bytes ({total_pages} pages)...")

	# Initialize SWD
	swd_connect()

	# Reset CPU into debug mode
	reset_into_debug()

	# Set VTOR to RAM
	write_word(0xE000ED08, 0x20000000)

	# Prepare flash
	flash_connect()
	flash_exit_xip()

	# Erase entire flash area needed
	flash_erase(FLASH_START, len(binary))

	flash_flush_cache()

	# Program each 4K page
	for page in range(total_pages):
		addr = FLASH_START + (page * PAGE_SIZE)
		page_data = binary[page * PAGE_SIZE : (page + 1) * PAGE_SIZE]
		print(f"Programming page {page+1}/{total_pages} at 0x{addr:08X}...")
		flash_program_page(addr, page_data)

	flash_flush_cache()
	flash_enter_xip()

	# Verify
	print("Verifying...")
	for page in range(total_pages):
		addr = FLASH_START + (page * PAGE_SIZE)
		page_data = binary[page * PAGE_SIZE : (page + 1) * PAGE_SIZE]
		verify_page(addr, page_data)
		print(f"Verified page {page+1}/{total_pages}")

	print("Flash complete and verified!")

	# Reset and run
	resume_cpu()
	print("Pico is running new firmware!")

def cleanup():
	lgpio.gpio_write(h, SWCLK_PIN, 0)
	lgpio.gpio_write(h, SWDIO_PIN, 0)
	lgpio.gpiochip_close(h)

	# Runs only when script is executed directly, not imported as module

def test_stage_1_reset_only():
	"""Stage 1: connect + reset + halt only. No flash touched."""
	swd_connect()
	reset_into_debug()
	write_word(0xE000ED08, 0x20000000)  # VTOR -> RAM
	print("Stage 1 PASSED: connect + reset + halt all worked")


def test_stage_2_lookup():
	"""Stage 2: stage 1, plus resolve a ROM function via table_lookup bootstrap."""
	swd_connect()
	reset_into_debug()
	write_word(0xE000ED08, 0x20000000)

	addr = lookup_rom_function("CX")
	print(f"CX (flash_enter_cmd_xip) resolved to: 0x{addr:08X}")
	if addr == 0 or addr > 0x00004000:
		print("WARNING: address looks implausible for boot ROM (expected < 0x4000)")
	else:
		print("Stage 2 PASSED: table_lookup bootstrap works")


def test_stage_3_trampoline_call():
	"""Stage 3: stage 2, plus one real call through the DT trampoline."""
	swd_connect()
	reset_into_debug()
	write_word(0xE000ED08, 0x20000000)

	flash_connect()  # exercises lookup_rom_function("DT") + call_rom_function()
	print("Stage 3 PASSED: DT trampoline + ROM call worked (connect_internal_flash ran)")

### ENTRY POINT ###
if __name__ == "__main__":
# sys.argv[0] = script name (pico_flash.py)
# sys.argv[1] = argument ("firmware.bin")

	if DEBUG:
		# Test GPIO before attempting SWD
		print("Testing GPIO...")
		lgpio.gpio_claim_output(h, SWDIO_PIN, 0)
		set_dio(1)
		time.sleep(0.1)
		val = lgpio.gpio_read(h, SWDIO_PIN)
		print(f"SWDIO set HIGH reads: {val}")
		set_dio(0)
		time.sleep(0.1)
		val = lgpio.gpio_read(h, SWDIO_PIN)
		print(f"SWDIO set LOW reads: {val}")

		print("Testing direction switch...")
		lgpio.gpio_claim_output(h, SWDIO_PIN, 0)
		set_dio(0)
		time.sleep(0.1)
		print(f"Output LOW: {lgpio.gpio_read(h, SWDIO_PIN)}")

		lgpio.gpio_claim_input(h, SWDIO_PIN)
		time.sleep(0.1)
		print(f"Input with pull-down (nothing connected): {lgpio.gpio_read(h, SWDIO_PIN)}")

		lgpio.gpio_claim_output(h, SWDIO_PIN, 0)
		set_dio(1)
		time.sleep(0.1)
		print(f"Output HIGH: {lgpio.gpio_read(h, SWDIO_PIN)}")

		lgpio.gpio_claim_input(h, SWDIO_PIN)
		time.sleep(0.1)
		print(f"Input with pull-down after HIGH: {lgpio.gpio_read(h, SWDIO_PIN)}")

		print("Starting in 10 seconds")
		time.sleep(10)


	if len(sys.argv) != 2: # Checks if only 1 argument provided plus script name (2 total)
		print("Usage: sudo python3 pico_flash.py firmware.bin")
		sys.exit(1)

	"""
	try:
		flash_binary(sys.argv[1]) # Pass filename argument to main
	except Exception as e:
		print(f"Error: {e}")
	finally:
		cleanup()
	"""

	try:
		# --- TESTING: comment/uncomment one line at a time, in order ---
		test_stage_1_reset_only()
		# test_stage_2_lookup()
		# test_stage_3_trampoline_call()

		# --- Once all three pass, switch back to the real thing: ---
		# flash_binary(sys.argv[1])
	except Exception as e:
		print(f"Error: {e}")
	finally:
		cleanup()
