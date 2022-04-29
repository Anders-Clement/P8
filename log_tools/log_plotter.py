import matplotlib.pyplot as plt
from import_hololens_log import importLogFolderToTrials
from log_dataclasses import FunctionLog, Position, PositionLog, Scene, Trial

def plotPositions(positions: list, show=True):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_title('hololensPosition')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    x = []
    y = []
    z = []
    for pos in positions:
        x.append(pos.hololensPosition.x)
        y.append(pos.hololensPosition.y)
        z.append(pos.hololensPosition.z)

    ax.plot3D(x,y,z)
    if show:
        plt.show()


def plotTargetPositions(positions: list, show=True):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_title('targetPosition')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    x = []
    y = []
    z = []
    for pos in positions:
        if not "Spatial" in pos.targetName:
            x.append(pos.targetPosition.x)
            y.append(pos.targetPosition.y)
            z.append(pos.targetPosition.z)

    ax.scatter(x,y,z)
    if show:
        plt.show()


def summarizeScene(scene: Scene) -> 'tuple[dict, dict]':
    functionCalls = {}
    objectsLookedAt = {}

    for functionCall in scene.functionLogs:
        if functionCall.functionName in functionCalls:
            functionCalls[functionCall.functionName] += 1
        else:
            functionCalls[functionCall.functionName] = 1

    for positionLog in scene.positionLogs:
        if positionLog.targetName in objectsLookedAt:
            objectsLookedAt[positionLog.targetName] += 1
        else:
            objectsLookedAt[positionLog.targetName] = 1

    functionCalls = dict(sorted(functionCalls.items(), key=lambda item: item[1]))
    objectsLookedAt = dict(sorted(objectsLookedAt.items(), key=lambda item: item[1]))
    return functionCalls, objectsLookedAt


if __name__ == '__main__':
    logFolder = "C:\\Users\\ander\\Desktop\\LOGS_HOLOLENS" 
    trials = importLogFolderToTrials(logFolder)
    for trial in trials:
        print(trial.fileName, [scene.name for scene in trial.scenes])
        for scene in trial.scenes:
            print(scene.name, "num logs: ", len(scene.rawLog))
            functionCalls, objectsLookedAt = summarizeScene(scene)
            print(functionCalls, '\n', objectsLookedAt)        
        print()