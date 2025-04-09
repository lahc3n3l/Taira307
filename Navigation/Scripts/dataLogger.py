import os, time
import machine

class DataLogger:
    def __init__(self, prefix="flight_log"):
        try:
            os.listdir()
        except OSError:
            print("Error: Filesystem not accessible")
            raise
            
        self.prefix = prefix
        self.filename = self._generate_filename()
        self._init_logfile()
        self.buffer = []
        self.buffer_size = 5  # Smaller buffer to reduce data loss risk
        
    def _generate_filename(self):
        timestamp = time.localtime()
        return f"{self.prefix}_{timestamp[0]:04d}{timestamp[1]:02d}{timestamp[2]:02d}_{timestamp[3]:02d}{timestamp[4]:02d}{timestamp[5]:02d}.csv"
    
    def _init_logfile(self):
        headers = [
            "timestamp",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz",
            "pitch", "roll", "yaw",
            "target_pitch", "target_roll", "target_flaps",
            "right_servo", "left_servo", "pitch_servo"
        ]
        try:
            with open(self.filename, "w") as f:
                f.write(",".join(headers) + "\n")
                f.flush()
                os.sync()  # Force sync to filesystem
        except OSError as e:
            print(f"Error creating log file: {e}")
            raise
            
    def log_data(self, data_dict):
        """Buffer the data and write when buffer is full."""
        try:
            self.buffer.append(data_dict)
            
            if len(self.buffer) >= self.buffer_size:
                self._write_buffer()
                self.buffer = []  # Clear buffer after write
                
        except Exception as e:
            print(f"Error in log_data: {e}")
            
    def _write_buffer(self):
        """Write buffered data to file with immediate sync."""
        try:
            with open(self.filename, "a") as f:
                for data_dict in self.buffer:
                    row = (f"{data_dict['timestamp']},{data_dict['ax']},{data_dict['ay']},{data_dict['az']},{data_dict['gx']},{data_dict['gy']},{data_dict['gz']},{data_dict['mx']},{data_dict['my']},{data_dict['mz']},{data_dict['pitch']},{data_dict['roll']},{data_dict['yaw']},{data_dict['target_pitch']},{data_dict['target_roll']},{data_dict['target_flaps']},{data_dict['right_servo']},{data_dict['left_servo']},{data_dict['pitch_servo']}\n")
                    f.write(row)
                f.flush()
                os.sync()  # Force sync to filesystem
        except Exception as e:
            print(f"Error writing buffer to file: {e}")
            
    def close(self):
        """Ensure all buffered data is written before closing."""
        try:
            if self.buffer:  # Write any remaining buffered data
                self._write_buffer()
            os.sync()  # Final sync
            print(f"Logging completed. Data saved to: {self.filename}")
        except Exception as e:
            print(f"Error during close: {e}")
