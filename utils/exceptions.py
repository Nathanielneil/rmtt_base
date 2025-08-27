class TelloConnectionError(Exception):
    def __init__(self, message="Tello连接错误"):
        self.message = message
        super().__init__(self.message)


class TelloControlError(Exception):
    def __init__(self, message="Tello控制错误"):
        self.message = message
        super().__init__(self.message)


class TelloSafetyError(Exception):
    def __init__(self, message="Tello安全机制触发"):
        self.message = message
        super().__init__(self.message)