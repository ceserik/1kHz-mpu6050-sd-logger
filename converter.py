import struct
import os
import glob

automatic = 1
input_file = "60.bin"
##################################### YOU HAVE TO CHANGE PATH TO YOUR SD CARD IF YOU WANT TO USE AUTMOATIC################################
if automatic == 1:
    sd_path = "/run/media/Riso/B585-A07D/"
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
    zero_count = 0  # Counter for consecutive zero samples
    log_stopped = False
    stop_sample = None
    prev_timestamp = None
    timestamp_gaps = []  # List to store (gap, sample_num, prev_ts, curr_ts)
    
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
        
        # Track timestamp differences
        if prev_timestamp is not None:
            gap = timestamp - prev_timestamp
            timestamp_gaps.append((gap, sample_num, prev_timestamp, timestamp))
        prev_timestamp = timestamp
        
        # Check if all IMU values are zero
        if ax == 0 and ay == 0 and az == 0 and gx == 0 and gy == 0 and gz == 0:
            zero_count += 1
            if zero_count == 10 and not log_stopped:
                log_stopped = True
                stop_sample = sample_num - 9  # The first zero sample
                print(f"\n*** LOG STOPPED at sample {stop_sample} (10 consecutive zero samples detected) ***\n")
        else:
            zero_count = 0  # Reset counter if non-zero sample found
        
        # Write to CSV
        f_out.write(f"{sample_num},{timestamp},{ax},{ay},{az},{gx},{gy},{gz}\n")
        sample_num += 1

if log_stopped:
    print(f"Converted {sample_num} samples to {output_file}")
    print(f"Log stopped at sample {stop_sample} ({stop_sample} valid samples before stopping)")
else:
    print(f"Converted {sample_num} samples to {output_file} (no stop detected)")

# Print 10 largest timestamp gaps
if timestamp_gaps:
    print(f"\n{'='*70}")
    print("10 LARGEST TIMESTAMP GAPS:")
    print(f"{'='*70}")
    timestamp_gaps.sort(reverse=True, key=lambda x: x[0])
    for i, (gap, sample, prev_ts, curr_ts) in enumerate(timestamp_gaps[:10], 1):
        print(f"{i:2d}. Gap: {gap:6d} ms at sample {sample:6d} (from {prev_ts} to {curr_ts})")
    print(f"{'='*70}\n")
