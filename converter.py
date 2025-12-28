import struct
import os
import glob

automatic = 1
input_file = "60.bin"
##################################### YOU HAVE TO CHANGE PATH TO YOUR SD CARD IF YOU WANT TO USE AUTMOATIC################################
if automatic == 1:
    sd_path = "/run/media/Riso/4331-0AE6/"
    bin_files = glob.glob(os.path.join(sd_path, "*.bin"))

    if not bin_files:
        print("No .bin files found!")
        exit(1)

    # Extract numbers from filenames and find the highest
    bin_files.sort(key=lambda x: int(os.path.basename(x).split('.')[0]))
    input_file = bin_files[-1]

# Extract the number from the binary filename
file_number = os.path.basename(input_file).split('.')[0]
output_file = f"output{file_number}.csv"

print(f"Converting {input_file}...")

with open(input_file, "rb") as f_in, open(output_file, "w") as f_out:
    # Write CSV header
    f_out.write("sample,timestamp_ms,ax,ay,az,gx,gy,gz\n")
    
    sample_num = 0
    while True:
        # Read 16 bytes (one sample: 4 timestamp + 6 accel + 6 gyro)
        bytes_read = f_in.read(16)
        if len(bytes_read) < 16:
            break
        
        # Debug: print first sample raw bytes
        if sample_num == 0:
            print(f"First sample raw bytes: {bytes_read.hex()}")
        
        # '>I' = big-endian unsigned int32 (timestamp)
        # '>hhhhhh' = big-endian signed int16 (6 values for IMU)
        timestamp = struct.unpack(">I", bytes_read[0:4])[0]
        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytes_read[4:16])
        
        # Debug: print first few samples
        if sample_num < 5:
            print(f"Sample {sample_num}: ts={timestamp}, ax={ax}, ay={ay}, az={az}, gx={gx}, gy={gy}, gz={gz}")
        
        # Write to CSV
        f_out.write(f"{sample_num},{timestamp},{ax},{ay},{az},{gx},{gy},{gz}\n")
        sample_num += 1

print(f"Converted {sample_num} samples to {output_file}")
