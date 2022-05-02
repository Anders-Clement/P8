from dataclasses import dataclass
import typing


@dataclass
class Position:
    """one position, x,y,z"""
    x: float
    y: float
    z: float

    def __init__(self, positionAsString) -> None:
        """create Position from string formatted as (0,0,0)"""
        fields = positionAsString.split(',')
        # remove left (
        self.x = float(fields[0][1:])
        self.y = float(fields[1])
        # remove right )
        self.z = float(fields[2][:-1])

    def toList(self) -> 'list[float]':
        return [self.x,self.y,self.z]

    def fromXYZ(x,y,z):
        return Position('(' + str(x) + ',' + str(y) + ',' + str(z) + ')')

    def __add__(self, other):
        return Position.fromXYZ(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Position.fromXYZ(self.x - other.x, self.y - other.y, self.z - other.z)

    def __abs__(self):
        return Position.fromXYZ(abs(self.x), abs(self.y), abs(self.z))


@dataclass
class PositionLog:
    """One timestamped log of hololens position, eyetracking target and eyetracking target position"""
    time: float
    hololensPosition: Position
    hololensOrientation: Position
    targetName: str
    targetPosition: Position

    def __init__(self, fields) -> None:
        self.time = fields[0]
        self.hololensPosition = Position(fields[1])
        self.hololensOrientation = Position(fields[2])
        hitName = fields[3]
        if "_v" in hitName:
            hitName = "robot_v"
        elif "_link" in hitName:
            hitName = "robot"
        elif "shelf" in hitName:
            hitName = "shelf"

        self.targetName = hitName
        self.targetPosition = Position(fields[4])


@dataclass
class FunctionLog:
    """One log of a function call"""
    time: float
    functionName: str
    additionalData: str = ""

    def __init__(self, fields) -> None:
        self.time = fields[0]
        self.functionName = fields[1]
        if len(fields) > 2:
            self.additionalData = fields[2:]


@dataclass
class SceneStatistic:
    totalHeadMovement: Position
    totalHeadRotation: Position

@dataclass
class TrialStatistic:
    pass


@dataclass
class Scene:
    """Logs from one scene, both raw and processed logs"""
    name: str
    rawLog: 'list[str]'
    positionLogs: typing.List[PositionLog]
    functionLogs: typing.List[FunctionLog]
    sceneStatistic: SceneStatistic = None

    def __init__(self, name, log) -> None:
        self.name = name
        self.rawLog = log
        while True:
            if len(self.rawLog) > 1:
                if "FUNCTION" in self.rawLog[0] and "IMAGE_TARGET_FOUND" in self.rawLog[0]:
                    self.rawLog.pop(0)
                    break
                else:
                    self.rawLog.pop(0)
            else:
                break
        print("=== WARNING: In Scene => check for scene end NOT IMPLEMENTED, using ALL data ===")
        self.positionLogs, self.functionLogs = self.__passLinesToPositionAndFunctionLogs(self.rawLog)


    def __passLinesToPositionAndFunctionLogs(self, lines: list) -> 'tuple[list[PositionLog], list[FunctionLog]]':
        """Helper function for parsing a log to functionlogs and positionlogs"""
        positionLogs = []
        functionLogs = []
        for line in lines:
            # remove \n 
            line = line[:-1]
            fields = line.split(';')
            if fields[1] == "FUNCTION":
                log = FunctionLog(fields[1:])
                functionLogs.append(log)
            else:
                log = PositionLog(fields)
                positionLogs.append(log)

        return positionLogs, functionLogs

@dataclass
class Trial:
    """Set of scenes, constituting all data from one participant"""
    scenes: typing.List[Scene]
    fileName: str
    folderName: str
    statistics: TrialStatistic = None


