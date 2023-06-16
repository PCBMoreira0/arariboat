from platformio.public import DeviceMonitorFilterBase

class MyFilter(DeviceMonitorFilterBase):
    NAME = "myfilter"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print("My filter has loaded")
        # buffer
        self.buffer = ""

    def tx(self, text):
        print(f"TX: {text}")
        return text

    def rx(self, text):
        self.buffer += text
        # read until first new line
        if "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if line.startswith("$GP"):
                #ignore GPS messages that are not GPRMC
                if line.startswith("$GPRMC"):
                    return f"{line}\n"
                return ""            
            return f"{line}\n"
        return ""
        