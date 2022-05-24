from log_dataclasses import Trial, Scene, FunctionLog, PositionLog, Position, SceneStatistic, TrialStatistic

def calculateMovements(positionLogs: 'list[PositionLog]') -> 'tuple[Position, Position]':
    totalHeadMovement = Position.fromXYZ(0,0,0)
    totalHeadRotation = Position.fromXYZ(0,0,0)
    
    lastMovementPosition = None
    lastMovementRotation = None
    for position in positionLogs:
        if lastMovementPosition is None:
            lastMovementPosition = position.hololensPosition
        else:
            totalHeadMovement += abs(position.hololensPosition - lastMovementPosition)
            lastMovementPosition = position.hololensPosition

        if lastMovementRotation is None:
            lastMovementRotation =  position.hololensOrientation
        else:
            nextRot = position.hololensOrientation
            subtracted = nextRot - lastMovementRotation
            # handle wraparound from high to low: 359 => 1 
            if subtracted.x < -180: subtracted.x += 360
            if subtracted.y < -180: subtracted.y += 360
            if subtracted.z < -180: subtracted.z += 360

            # handle wraparound from low to high: 1 => 359
            if subtracted.x > 180: subtracted.x = 360 - subtracted.x
            if subtracted.x > 180: subtracted.x = 360 - subtracted.x
            if subtracted.x > 180: subtracted.x = 360 - subtracted.x

            totalHeadRotation += abs(subtracted)
            lastMovementRotation =  nextRot

    return totalHeadMovement, totalHeadRotation


def sumObjectsLookedAt(scene: Scene, output: 'dict[str,int]' = dict()) -> 'dict[str, int]':
    output = {}

    for positionLog in scene.positionLogs:
        if positionLog.targetName in output:
            output[positionLog.targetName] += 1
        else:
            output[positionLog.targetName] = 1
    
    return output


def calcSceneStatistics(scene: Scene) -> SceneStatistic:
    totalHeadMovement, totalHeadRotation = calculateMovements(scene.positionLogs)
    objectsLookedAt = sumObjectsLookedAt(scene)
    startTime = scene.positionLogs[0].time
    endTime = scene.positionLogs[-1].time
    duration = float(endTime) - float(startTime)
    numNearInteractions = 0
    numFarInteractions = 0

    for functionLog in scene.functionLogs:
        if "ManipulationStarted" in functionLog.functionName:
            if functionLog.additionalData[4] == 'True':
                numNearInteractions += 1
            else:
                numFarInteractions += 1

    numInteractions = numNearInteractions + numFarInteractions
    return SceneStatistic(totalHeadMovement, totalHeadRotation, objectsLookedAt, 
                            duration, numNearInteractions, numFarInteractions, numInteractions
                        )
   

def calcTrialStatistics(trial: Trial) -> TrialStatistic:
    totalDuration = 0
    totalNearInteractions = 0
    totalFarInteractions = 0
    totalRotation = Position.fromXYZ(0,0,0)
    totalMovement = Position.fromXYZ(0,0,0)
    for scene in trial.scenes:
        totalDuration += scene.sceneStatistic.duration
        totalNearInteractions += scene.sceneStatistic.numNearInteractions
        totalFarInteractions += scene.sceneStatistic.numFarInteractions
        totalRotation += scene.sceneStatistic.totalHeadRotation
        totalMovement += scene.sceneStatistic.totalHeadMovement

    totalInteractions = totalNearInteractions + totalFarInteractions
    statistic = TrialStatistic(totalInteractions, totalNearInteractions,
                                 totalFarInteractions, totalDuration, totalRotation, totalMovement)
    return statistic


def calculateStatisticsOnTrials(trials: 'list[Trial]') -> None:
    for trial in trials:
        for scene in trial.scenes:
            scene.sceneStatistic = calcSceneStatistics(scene)
        trial.trialStatistic = calcTrialStatistics(trial)


if __name__ == '__main__':
    from import_hololens_log import importLogFolderToTrials
    logFolder = "C:\\Users\\ander\\Desktop\\final_logs_2d" 
    trials = importLogFolderToTrials(logFolder, prunePlayScene=True)

    calculateStatisticsOnTrials(trials)
    for trial in trials:
        print(trial.fileName)
        for scene in trial.scenes:
            print(scene.name, scene.sceneStatistic)
            print()
        print("===============================================")