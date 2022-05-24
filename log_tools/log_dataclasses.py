from math import sqrt
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

    def fromXYZ(x: float, y: float, z: float):
        return Position('(' + str(x) + ',' + str(y) + ',' + str(z) + ')')

    def norm(self) -> float:
        return sqrt(self.x**2 + self.y**2 + self.z**2)

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
        self.time = float(fields[0])
        self.hololensPosition = Position(fields[1])
        self.hololensOrientation = Position(fields[2])

        hitName = fields[3]
        UINames = ["corner", "ContentBackPlate", "Delete", 
                    "SpawnPlace", "SpawnPick", "SpawnWP", "Sequencing",
                    "metal_shelf", "placePrefab", "pickPrefab", "waypointPrefab",
                    "Waypoint", "Pick", "Place", "Trash"]

        if "_v" in hitName:
            hitName = "Robot_v"
        elif "_link" in hitName:
            hitName = "Robot"
        elif "knuckle" in hitName or "finger" in hitName:
            hitName = "Gripper"
        elif "SpatialMesh" in hitName:
            hitName = "BG"
        elif "Background" in hitName:
            hitName = "BG"
        elif "SequencingSphere" in hitName:
            hitName = "Sequen."
        elif hitName == "pick":
            hitName = "Pick"
        elif hitName == "waypoint":
            hitName = "Waypoint"
        elif hitName == "place":
            hitName = "Place"
        elif hitName == "WP":
            hitName = "Waypoint"
        elif True in [name in hitName for name in UINames]:
            hitName = "UI"

        

        self.targetName = hitName
        self.targetPosition = Position(fields[4])


@dataclass
class FunctionLog:
    """One log of a function call"""
    time: float
    functionName: str
    additionalData: str = ""

    def __init__(self, fields) -> None:
        self.time = float(fields[0])
        self.functionName = fields[2]
        if len(fields) > 3:
            self.additionalData = fields[3:]


@dataclass
class SceneStatistic:
    totalHeadMovement: Position
    totalHeadRotation: Position
    objectsLookedAt: 'dict[str,int]'
    duration: float
    numNearInteractions: int
    numFarInteractions: int
    numInteractions: int


@dataclass
class TrialStatistic:
    numInteractions: int
    numNearInteractions: int
    numFarInteractions: int
    duration: float
    totalRotation: Position
    totalMovement: Position


@dataclass
class Scene:
    """Logs from one scene, both raw and processed logs"""
    name: str
    rawLog: 'list[str]'
    positionLogs: typing.List[PositionLog]
    functionLogs: typing.List[FunctionLog]
    sceneStatistic: SceneStatistic = None

    def __init__(self, name, log) -> None:
        """Scene log from scenename, and the raw CSV log lines"""
        self.name = name

        # run through log, delete data until scene start is found (IMAGE_TARGET_FOUND) since it is in wrong frame
        start_index = 0
        for i, logEntry in enumerate(log):
            if "FUNCTION" in log[i] and "IMAGE_TARGET_FOUND" in log[i]:
                start_index = i
                break # should only be one IMAGE_TARGET_FOUND, if present
        self.rawLog = log[start_index:]
        # while True:
        #     if len(self.rawLog) > 1:
        #         if "FUNCTION" in self.rawLog[0] and "IMAGE_TARGET_FOUND" in self.rawLog[0]:
        #             self.rawLog.pop(0)
        #             break
        #         else:
        #             self.rawLog.pop(0)
        #     else:
        #         break
        # if self.name == "finalPlayscene":
        #     self.positionLogs, self.functionLogs = self.__passLinesToPositionAndFunctionLogs(self.rawLog)
        # elif self.name == "finalTesting":
        #     self.positionLogs, self.functionLogs = self.__passLinesToPositionAndFunctionLogs(self.rawLog)
        # for f in self.functionLogs:
        #     print(f)
        # print("=== WARNING: In Scene => check for scene end NOT IMPLEMENTED, using ALL data ===")
        self.positionLogs, self.functionLogs = self.__passLinesToPositionAndFunctionLogs(self.rawLog)


    def __passLinesToPositionAndFunctionLogs(self, lines: list) -> 'tuple[list[PositionLog], list[FunctionLog]]':
        """Helper function for parsing a log to functionlogs and positionlogs"""
        positionLogs = []
        functionLogs = []
        for line in lines:
            # remove \n 
            if line[-1] is '\n': 
                line = line[:-1]
            fields = line.split(';')
            fields = [field for field in fields if field is not '']
            if fields[1] == "FUNCTION":
                log = FunctionLog(fields)
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
    trialStatistic: TrialStatistic = None

    def convertBoxNameToBoxType(self):
        """Converts hitname = box # into a hitname corresponding to the box type (WP, pick, place)"""
        # first discover all boxes, and save their name (number of box, from 0=>max(num boxes) and type
        boxes = {}
        for scene in self.scenes:
            for functionLog in scene.functionLogs:
                if "SpawnBoxOnShelf" in functionLog.functionName:
                    boxName = functionLog.additionalData[0]
                    boxes[len(boxes)] = boxName
                elif "Spawn" in functionLog.functionName:
                    boxName = functionLog.functionName[5:] # trim the 'Spawn'

                    # trim the rest:
                    if "WP" in boxName:
                        boxName = boxName[:2]
                    elif "Place" in boxName:
                        boxName = boxName[:5]
                    elif "Pick" in boxName: 
                        boxName = boxName[:4]

                    boxes[len(boxes)] = boxName

        # now change their targetname based on type
        for scene in self.scenes:
            for positionLog in scene.positionLogs:
                if "Box " in positionLog.targetName:
                    index = int(positionLog.targetName[4:])
                    positionLog.targetName = boxes[index]
                    pass


