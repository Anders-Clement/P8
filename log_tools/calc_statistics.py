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
            

def calcSceneStatistics(scene: Scene) -> SceneStatistic:
    totalHeadMovement, totalHeadRotation = calculateMovements(scene.positionLogs)
    return SceneStatistic(totalHeadMovement, totalHeadRotation)


def calcStatistics(trial: Trial):
    for scene in trial.scenes:
        scene.sceneStatistic = calcSceneStatistics(scene)


def calculateStatisticsOnTrials(trials: 'list[Trial]') -> None:
    for trial in trials:
        calcStatistics(trial)

if __name__ == '__main__':
    from import_hololens_log import importLogFolderToTrials
    logFolder = "C:\\Users\\ander\\Desktop\\LOGS_HOLOLENS" 
    trials = importLogFolderToTrials(logFolder)

    calculateStatisticsOnTrials(trials)
    for trial in trials:
        for scene in trial.scenes:
            print(scene.name, scene.sceneStatistic.totalHeadMovement, scene.sceneStatistic.totalHeadRotation)
        print()