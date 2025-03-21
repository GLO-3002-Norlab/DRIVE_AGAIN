class RobotData:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    timestamp: int
    command_id: float

    def __getstate__(self):
        return self.__dict__

    def __setstate__(self, d):
        self.__dict__ = d

    @staticmethod
    def from_state(s: dict) -> "RobotData":
        res = RobotData()
        res.__setstate__(s)
        return res
