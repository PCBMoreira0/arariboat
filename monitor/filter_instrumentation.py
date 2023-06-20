from platformio.public import DeviceMonitorFilterBase

class MyFilter(DeviceMonitorFilterBase):
    NAME = "instrumentation"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        print("Instrumentation filter has loaded")
        self.buffer = ""
    
    def tx(self, text):
        print(f"TX: {text}")
        return text
    
    def rx(self, text):
        self.buffer += text
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            if line.startswith("[Instrumentation"):
                return f"{line}\n"
            return ""
        return ""