import csv
import os

def process_trajectory(input_file, output_file):
    if not os.path.exists(input_file):
        print(f"Error: {input_file} does not exist.")
        return

    with open(input_file, mode='r', newline='') as infile:
        reader = csv.reader(infile)
        header = next(reader)
        
        # Read the first data row
        try:
            first_row = next(reader)
        except StopIteration:
            print("Error: Input file is empty after header.")
            return
            
        # Convert first row to floats for comparison
        temp = [float(val) for val in first_row]
        
        # We will record the header and the first row in the output
        recorded_rows = [header, first_row]
        
        for row in reader:
            if not row:
                continue
            
            # Convert current row to floats
            current_vals = [float(val) for val in row]
            
            # Check if any dimension difference > 0.1
            should_record = False
            for i in range(len(current_vals)):
                if abs(current_vals[i] - temp[i]) > 0.1:
                    should_record = True
                    break
            
            if should_record:
                recorded_rows.append(row)
                temp = current_vals # Update temp to the recorded row
                
    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerows(recorded_rows)
    
    print(f"Processed {input_file} -> {output_file}")
    print(f"Original rows: {reader.line_num}")
    print(f"Filtered rows: {len(recorded_rows)}")

if __name__ == "__main__":
    input_path = "/home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/traj4.csv"
    output_path = "/home/xiaozy24/dual_panda_ws/src/multipanda_ros2/tools/traj5.csv"
    process_trajectory(input_path, output_path)
