import struct
import os
import glob

# Find the highest numbered .bin file in the SD card directory
sd_path = "/run/media/Riso/4331-0AE6/"
bin_files = glob.glob(os.path.join(sd_path, "*.bin"))

if not bin_files:
    print("No .bin files found!")
    exit(1)

# Extract numbers from filenames and find the highest
bin_files.sort(key=lambda x: int(os.path.basename(x).split('.')[0]))
input_file = bin_files[-1]
output_file = "output.txt"

print(f"Converting {input_file}...")

with open(input_file, "rb") as f_in, open(output_file, "w") as f_out:
    # Write CSV header
    f_out.write("sample,ax,ay,az,gx,gy,gz\n")
    
    sample_num = 0
    while True:
        # Read 12 bytes (one sample: ax, ay, az, gx, gy, gz)
        bytes_read = f_in.read(12)
        if len(bytes_read) < 12:
            break
        
        # '>hhhhhh' = big-endian signed int16 (6 values)
        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytes_read)
        
        # Write to CSV
        f_out.write(f"{sample_num},{ax},{ay},{az},{gx},{gy},{gz}\n")
        sample_num += 1

print(f"Converted {sample_num} samples to {output_file}")
