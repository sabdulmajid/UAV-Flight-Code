from pymavlink import mavutil

# Connect to the SITL simulator
connection_string = 'tcp:127.0.0.1:5760'  # Update the IP address and port if necessary
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message to ensure connection is established
while True:
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        break

# Print some basic information about the vehicle
print(f"Autopilot: {msg.autopilot}")
print(f"System status: {msg.system_status}")

vehicle.close()


"""
This code snippet appears to be MIPS assembly code. Let's go through it step by step to understand its purpose and functionality.

1. `lis $8`
This instruction loads the immediate value `0x7` (which is the hexadecimal representation of the decimal number 7) into register `$8`. The `lis` instruction loads the upper 16 bits of the immediate value, and the lower 16 bits will be loaded later.

2. `.word 0x7`
This is a directive used in assembly code to allocate 4 bytes of memory and initialize it with the value `0x7`. This memory allocation will store the number 7 in the last possible spot in the array.

3. `lis $9`
Similar to the first step, this instruction loads the immediate value `4` into register `$9`. It is loading the upper 16 bits of the immediate value.

4. `.word 4`
This directive allocates 4 bytes of memory and initializes it with the value `4`. This memory allocation indicates the number of elements in the array. The code assumes that there are less than 2^20 (220) elements.

5. `mult $2, $9`
This instruction multiplies the value in register `$2` (which holds the number of elements) by the value in register `$9` (which holds the value 4). The result of the multiplication is a 64-bit value, and it is stored in two registers: `$HI` (the upper 32 bits) and `$LO` (the lower 32 bits).

6. `mflo $3`
This instruction moves the lower 32 bits of the multiplication result from register `$LO` to register `$3`. `$3` will now hold the total size (in bytes) required to reach the last element in the array.

7. `add $3, $3, $1`
This instruction adds the value in register `$1` (which contains the address of the array) to the value in register `$3` (which holds the total size). The result of the addition will be the address of the last element in the array.

8. `sw $8, -4($3)`
This instruction stores the value in register `$8` (which is 7) into memory. The value is stored at the address calculated in the previous step, but with an offset of -4 bytes. The negative offset is used to reach the last possible spot in the array, assuming each element in the array occupies 4 bytes.

9. `jr $31`
This instruction is a jump register instruction that returns control flow to the instruction at the address stored in register `$31`. It effectively ends the execution of this code snippet.

In summary, this code snippet takes an array's address in register `$1` and the number of elements in the array in register `$2`. It calculates the address of the last element in the array by multiplying the number of elements by 4 (assuming each element occupies 4 bytes) and adding it to the array's base address. Finally, it stores the value 7 in the last possible spot in the array.
"""