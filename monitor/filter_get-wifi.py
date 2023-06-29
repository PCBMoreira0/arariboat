from platformio.public import DeviceMonitorFilterBase

class MyFilter(DeviceMonitorFilterBase):
    NAME = "get-wifi"
    # Filter to check serial monitor to check for line that has "IP" in order to get wifi IP address
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
        self.buffer = ""
    
    def tx(self, text):
        print(f"TX: {text}")
        return text
    
    def rx(self, text):
        self.buffer += text
        if "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if "IP" in line:
                with open("last-wifi.txt", "w") as filewriter:
                    filewriter.write(line)
        return text