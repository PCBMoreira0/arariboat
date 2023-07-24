from platformio.public import DeviceMonitorFilterBase

# Make terminal output color green
class MyFilter(DeviceMonitorFilterBase):
    NAME = "color-orange"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print("Orange color filter has loaded")
        self.buffer = ""
    
    def tx(self, text):
        print(f"TX: {text}")
        return text
    
    def rx(self, text):
        self.buffer += text
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            return f"\033[38;5;208m{line}\033[0m\n"
        return ""

        