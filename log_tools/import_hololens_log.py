from log_dataclasses import Scene, Trial
import os

def importLogFileToScenes(filename: str) -> 'list[Scene]':
    """Imports a csv log file, and stores the data in a list of Scene dataclasses"""
    with open(filename, 'r') as f:
        lines = f.readlines()

    scenes: list[Scene] = []
    sceneName = ""
    sceneLog = []
    numTasks = 0
    for line in lines:
        #remove trailing \n
        line = line[:-1]
        fields = line.split(';')
        if fields[1] == "FUNCTION":
            if fields[2] == "SceneChangeTo":
                if sceneName != "":
                    scenes.append(Scene(sceneName, sceneLog))
                sceneName = fields[3]
                sceneLog = []
                continue
            elif  fields[2] == "END_TASK":
                sceneName = 'task_' + str(numTasks)
                scenes.append(Scene(sceneName, sceneLog[1:])) # trim FUNCTION log END_TASK before making scene
                numTasks += 1
                sceneLog = []

        sceneLog.append(line)
        
    # also save last scene, only for early tests
    if len(sceneLog) > 1:
        scenes.append(Scene(sceneName, sceneLog))

    return scenes


def importLogFolderToTrials(logFolder: str, prunePlayScene: bool = True) -> 'list[Trial]':
    list_of_files = []
    for root, dirs, files in os.walk(logFolder):
        for file in files:
            if ".csv" in file[-4:]:
                list_of_files.append([os.path.join(root,file), root])

    trials: 'list[Trial]' = []
    for logFile in list_of_files:
        scenes = importLogFileToScenes(logFile[0])
        trials.append(Trial(scenes, logFile[0], logFile[1]))

    if prunePlayScene:
        for trial in trials:
            trial.scenes = [scene for scene in trial.scenes if 'finalPlayscene' not in scene.name and 'ShelfPlayScene' not in scene.name]

    return trials