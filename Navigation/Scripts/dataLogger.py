import os

class DataLogger:
    def __init__(self, prefix="flight_log"):
        self.prefix = prefix
        self.filename = self._generate_filename()
        self._init_logfile()
        
    def _generate_filename(self):
        """Generate a unique filename with timestamp."""
        timestamp = time.localtime()
        return f"{self.prefix}_{timestamp[0]:04d}{timestamp[1]:02d}{timestamp[2]:02d}_{timestamp[3]:02d}{timestamp[4]:02d}{timestamp[5]:02d}.csv"
    
    def _init_logfile(self):
        """Initialize the log file with headers."""
        headers = [
            "timestamp",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "pitch", "roll", "yaw",
            "target_pitch", "target_roll",
            "right_servo", "left_servo", "pitch_servo"
        ]
        with open(self.filename, "w") as f:
            f.write(",".join(headers) + "\n")
            
    def log_data(self, data_dict):
        """Log a single row of data to the CSV file."""
        try:
            with open(self.filename, "a") as f:
                f.write(
                    f"{data_dict['timestamp']},"
                    f"{data_dict['ax']},{data_dict['ay']},{data_dict['az']},"
                    f"{data_dict['gx']},{data_dict['gy']},{data_dict['gz']},"
                    f"{data_dict['pitch']},{data_dict['roll']},{data_dict['yaw']},"
                    f"{data_dict['target_pitch']},{data_dict['target_roll']},"
                    f"{data_dict['right_servo']},{data_dict['left_servo']},{data_dict['pitch_servo']}\n"
                )
        except Exception as e:
            print(f"Error writing to log file: {e}")
            
    def close(self):
        """Cleanup method if needed."""
        print(f"Logging completed. Data saved to: {self.filename}")