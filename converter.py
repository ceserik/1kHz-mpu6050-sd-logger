import struct

input_file = "36.bin"
output_file = "output.txt"

with open(input_file, "rb") as f_in, open(output_file, "w") as f_out:
    # Write CSV header
    f_out.write("sample,ax,ay,az\n")
    
    sample_num = 0
    while True:
        # Read 6 bytes (one sample: ax, ay, az)
        bytes_read = f_in.read(6)
        if len(bytes_read) < 6:
            break
        
        # '>hhh' = big-endian signed int16 (3 values)
        ax, ay, az = struct.unpack(">hhh", bytes_read)
        
        # Write to CSV
        f_out.write(f"{sample_num},{ax},{ay},{az}\n")
        sample_num += 1

print(f"Converted {sample_num} samples to {output_file}")
