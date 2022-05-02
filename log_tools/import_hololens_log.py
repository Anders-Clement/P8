from log_dataclasses import Scene, Trial
import os

def importLogFileToScenes(filename: str) -> 'list[Scene]':
    """Imports a csv log file, and stores the data in a list of Scene dataclasses"""
    with open(filename, 'r') as f:
        lines = f.readlines()

    scenes: list[Scene] = []
    sceneName = ""
    sceneLog = []
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

        sceneLog.append(line)

    return scenes


def importLogFolderToTrials(logFolder: str) -> 'list[Trial]':
    list_of_files = []
    for root, dirs, files in os.walk(logFolder):
        for file in files:
            list_of_files.append([os.path.join(root,file), root])

    trials = []
    for logFile in list_of_files:
        scenes = importLogFileToScenes(logFile[0])
        trials.append(Trial(scenes, logFile[0], logFile[1]))

    return trials