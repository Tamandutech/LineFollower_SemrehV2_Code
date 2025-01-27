def offset_map_data(line_number, offset_value):
    with open("C:\\Users\\henri\\OneDrive\\Documentos\\PlatformIO\\Projects\\LineFollower_SemrehV2_Code\\data\\Map_Data_Original.txt", "r") as input_file:
        lines = input_file.readlines()
    
    with open("C:\\Users\\henri\\OneDrive\\Documentos\\PlatformIO\\Projects\\LineFollower_SemrehV2_Code\\data\\Map_Data.txt", "w") as output_file:
        line_count = 0
        for line in lines:
            line_count += 1
            if line_count >= line_number:
                values = [float(value) for value in line.strip().split(",")]
                values[2] += offset_value
                values[5] += offset_value
                values[6] += offset_value
                output_file.write(",".join(map(str, values)) + "\n")
            else:
                output_file.write(line)

if __name__ == "__main__":
    offset_map_data(42, 200)
