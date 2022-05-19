import matplotlib.pyplot as plt
import numpy as np
from import_hololens_log import importLogFolderToTrials
from log_dataclasses import PositionLog, Scene, Trial
from calc_statistics import sumObjectsLookedAt, calculateStatisticsOnTrials


def plotPositions(positions: 'list[PositionLog]', show=True) -> None:
    """May be useful, plots positions of the hololens in space"""
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


def plotTargetPositions(positions: 'list[PositionLog]', show=True) -> None:
    """May be useful, plots positions of the objects the user has looked at"""
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
    """Not used anymore"""
    functionCalls = {}

    for functionCall in scene.functionLogs:
        if functionCall.functionName in functionCalls:
            functionCalls[functionCall.functionName] += 1
        else:
            functionCalls[functionCall.functionName] = 1

    objectsLookedAt = sumObjectsLookedAt(scene)
    functionCalls = dict(sorted(functionCalls.items(), key=lambda item: item[1]))
    objectsLookedAt = dict(sorted(objectsLookedAt.items(), key=lambda item: item[1]))
    return functionCalls, objectsLookedAt


def sumOverTrialsObjectsLookedAt(trials: 'list[Trial]', sort: bool = True, plot = True) -> 'dict[str, int]':
    """Not used anymore"""
    objectsLookedAt = {}
    for trial in trials:
        for scene in trial.scenes:
            if "task_" in scene.name:
                objectsLookedAt = sumObjectsLookedAt(scene, objectsLookedAt)
    if sort:
        objectsLookedAt = dict(sorted(objectsLookedAt.items(), key=lambda item: item[1]))

    if plot:
        names = list(objectsLookedAt.keys())
        values = list(objectsLookedAt.values())

        plt.bar(range(len(objectsLookedAt)), values, tick_label=names)
        plt.show()

    return objectsLookedAt


def plotInteractionsPerTask(trials: 'list[Trial]', useMedian=False) -> None:
    """Makes a plot of the total number of far/near interactions per task"""
    # extract the data
    nearInteractions = [[],[],[]]
    farInteractions = [[],[],[]]
    for trial in trials:
        for scene in trial.scenes:
            if 'task_0' in scene.name:
                index = 0
            elif 'task_1' in scene.name:
                index = 1
            elif 'task_2' in scene.name:
                index = 2
            else:
                print("Unknown scene name! skipping...")
                continue
                
            nearInteractions[index].append(scene.sceneStatistic.numNearInteractions)
            farInteractions[index].append(scene.sceneStatistic.numFarInteractions)

    if useMedian:
        nearInteractions = np.median(np.array(nearInteractions),axis=1)
        farInteractions = np.median(np.array(farInteractions), axis=1)
    else:
        nearInteractions = np.sum(np.array(nearInteractions),axis=1)
        farInteractions = np.sum(np.array(farInteractions), axis=1)

    N = 3
    # menMeans = (20, 35, 30, 35, 27)
    # womenMeans = (25, 32, 34, 20, 25)
    width = 0.35
    ind = np.arange(N) + 1 # the x locations for the groups
    ind2 = [x + width for x in ind]

    fig, ax = plt.subplots(1)
    #ax = fig.add_axes([0,0,1,1])
    ax.bar(ind, nearInteractions, width, color='r')
    ax.bar(ind2, farInteractions, width, color='b')
    ax.set_ylabel('Number of interactions')
    #ax.set_xlabel('Task')
    #ax.set_title('Scores by group and gender')
    ax.set_xticks([x + width/2.0 for x in ind]) #, labels=['Task 1', 'Task 2', 'Task 3'])
    ax.set_xticklabels(['Task 1', 'Task 2', 'Task 3'])
    if not useMedian:
        ax.set_yticks(np.arange(0, 175, 20))
    ax.legend(labels=['Near interaction', 'Far interaction'])
    if useMedian:
        title = "Median number of far/near interactions per task"
    else:
        title='Total number of far/near interactions per task'
    fig.suptitle(title, fontsize=16)
    global logFolder
    if useMedian:
        fig.savefig(logFolder + '\\plotInteractionsPerTaskMedian.pdf')
    else:
        fig.savefig(logFolder + '\\plotInteractionsPerTask.pdf')


def plotInteractionsPerUser(trials: 'list[Trial]') -> None:
    """Makes a plot of the total number of far/near interactions per user over all tasks"""
    nearInteractions = []
    farInteractions = []
    for trial in trials:
        nearInteractions.append(trial.trialStatistic.numNearInteractions)
        farInteractions.append(trial.trialStatistic.numFarInteractions)
    N = len(trials)
    fig, ax = plt.subplots(1)
    ind = np.arange(N) +1
    width = 0.35
    ax.bar(ind, nearInteractions, width, color='r')
    ax.bar(ind, farInteractions, width,bottom=nearInteractions, color='b')
    ax.set_ylabel('Number of interactions')
    ax.set_xlabel('Participant')
    #ax.set_title('Scores by group and gender')
    ax.set_xticks(ind) #, labels=['Task 1', 'Task 2', 'Task 3'])
    #ax.set_xticklabels(['Task 1', 'Task 2', 'Task 3'])
    ax.set_yticks(np.arange(0, 100, 10))
    ax.legend(labels=['Near interaction', 'Far interaction'])
    fig.suptitle('Far/near interaction per user over all tasks', fontsize=16)
    global logFolder
    fig.savefig(logFolder + '\\plotInteractionsPerUser.pdf')
    

def checkIfAlwaysOneHandInteraction(trials: 'list[Trial]') -> bool:
    """Function to check if manipulation started is always followed by manipulationended 
        also checking for if deletetoggle is on, and ignoring those manipulationstarted, 
        as they do not trigger a manipulation ended when the box is deleted."""
    alwaysOneHanded = True
    for trial in trials:
        for scene in trial.scenes:
            interacting = False
            deleteToggle = False
            numStarts = 0
            numEnds = 0
            for log in scene.functionLogs:
                if "ManipulationStarted" in log.functionName:
                    if not deleteToggle:
                        numStarts += 1
                        if interacting == False:
                            interacting = True
                        else:
                            print("already interacting!", scene.name, trial.fileName)
                elif "ManipulationEnded" in log.functionName:
                    numEnds += 1
                    if interacting:
                        interacting = False
                    else:
                        print('Ended, but not interacting?')
                elif "DeleteToggle" in log.functionName:
                    if "True" in log.additionalData:
                        deleteToggle = True
                    elif "False" in log.additionalData:
                        deleteToggle = False
                    else:
                        print("deletetoggle: unknown additionalData: ", log.additionalData)

            # print('starts == ends: ', numStarts == numEnds, numStarts, numEnds)
            if numStarts is not numEnds:
                alwaysOneHanded = False
    return alwaysOneHanded


def plotObjectsLookedAtPerTask(trials: 'list[Trial]') -> None:
    """Bar chart of objects looked at, with a plot for each task."""
    task_0_objects = {}
    task_1_objects = {}
    task_2_objects = {}
    for trial in trials:
        for scene in trial.scenes:
            if "task_0" in scene.name:
                task_0_objects = sumObjectsLookedAt(scene, task_0_objects)
            if "task_1" in scene.name:
                task_1_objects = sumObjectsLookedAt(scene, task_1_objects)
            if "task_2" in scene.name:
                task_2_objects = sumObjectsLookedAt(scene, task_2_objects)

    # ensure all keys are in all dicts, to plot them as a zero
    for key in task_0_objects.keys():
        if key not in task_1_objects.keys():
            task_1_objects[key] = 0
        if key not in task_2_objects.keys():
            task_2_objects[key] = 0

    for key in task_1_objects.keys():
        if key not in task_0_objects.keys():
            task_0_objects[key] = 0
        if key not in task_2_objects.keys():
            task_2_objects[key] = 0

    for key in task_2_objects.keys():
        if key not in task_1_objects.keys():
            task_1_objects[key] = 0
        if key not in task_0_objects.keys():
            task_0_objects[key] = 0


    # sort by maximum value over tasks
    # sortKeyFunc = lambda item: max(task_0_objects[item[0]],task_1_objects[item[0]], task_2_objects[item[0]])
    # sort by maximum value of task 1
    sortKeyFunc = lambda item: item[1]
    task_0_objects = dict(sorted(task_0_objects.items(), key=sortKeyFunc))
    
    N = len(task_0_objects)
    width = .35

    fig, ax = plt.subplots(1)
    ind = np.arange(N) - width
    ind = [x + i*width for i, x in enumerate(ind)]
    ind2 = [x + width for x in ind]
    ind3 = [x + width for x in ind2]

    task1 = [task_0_objects[key] for key in task_0_objects.keys()]
    task2 = [task_1_objects[key] for key in task_0_objects.keys()]
    task3 = [task_2_objects[key] for key in task_0_objects.keys()]
    
    ax.bar(ind, task1, width)
    ax.bar(ind2, task2, width, tick_label=list(task_0_objects.keys()))
    ax.bar(ind3, task3, width)
    ax.set_ylim((0,1000))
    ax.set_ylabel('Number of observations')
    ax.legend(['Task 1', 'Task 2', 'Task 3'])
    fig.suptitle('Objects seen by the participants', fontsize=16)
    fig.tight_layout()
    global logFolder
    fig.savefig(logFolder + '\\plotObjectsLookedAtPerTask.pdf')


def getDurationsOfTasks(trials: 'list[Trial]') -> 'tuple[list[float], list[float], list[float]]':
    """Extracts the total durations of each task over all trials"""
    task_0_durations = []
    task_1_durations = []
    task_2_durations = []
    for trial in trials:
        for scene in trial.scenes:
            if "task_0" in scene.name:
                task_0_durations.append(scene.sceneStatistic.duration)
            if "task_1" in scene.name:
                task_1_durations.append(scene.sceneStatistic.duration)
            if "task_2" in scene.name:
                task_2_durations.append(scene.sceneStatistic.duration)
    return [task_0_durations, task_1_durations, task_2_durations]


def taskDurationStatistics(trials: 'list[Trial]'):
    """Calculates median and average completion time for each task"""
    durationOfTasks = getDurationsOfTasks(trials)
    print("==== Duration Statistics ====")
    for i in range(3):
        durationOfTasks[i].append(np.median(np.array(durationOfTasks[i])))
        durationOfTasks[i].append(np.average(np.array(durationOfTasks[i])))
        print("============== TASK " + str(i) + " ==============")
        print(durationOfTasks[i][:-2])
        print('median: ', durationOfTasks[i][-2])
        print('average: ', durationOfTasks[i][-1])
        print('sum: ', np.sum(durationOfTasks[i][:-2]))
    print('==== // Duration Statistics ====')
    data = np.array(durationOfTasks)
    np.savetxt('taskDuration.csv', data, header="durations; median; average; from top: task_0 to task_2", delimiter=';')
    return durationOfTasks


def plotDurationOfTasks(durations: 'list[float]'):
    """Makes a bar plot of average duration over all participants for each task"""
    fig, ax = plt.subplots(1)
    data = np.array(durations)[:,-1]
    ax.bar([1,2,3], data, width=.35, tick_label=["Task 1", "Task 2", "Task 3"])
    ax.set_ylabel('[s]')
    fig.suptitle('Average duration of tasks', fontsize=16)
    global logFolder
    fig.savefig(logFolder + '\\plotDurationOfTasks.pdf')


def getDurationOfInteractions(trials: 'list[Trial]') -> 'None | list[list[list[float]]]':

    if not checkIfAlwaysOneHandInteraction(trials):
        print("Not always one handed interaction!")
        return None
    else:
        participantNearInteractions = []
        participantFarInteractions = []
        for trial in trials:
            taskNearInteractions = []
            taskFarInteractions = []
            for scene in trial.scenes:
                nearInteractions = []
                farInteractions = []
                manipulating = ""
                manipulationType = ""
                for log in scene.functionLogs:
                    if "ManipulationStarted" in log.functionName:
                        if manipulating is not "":
                            print("Got manipulation started, but am already manipulating!")
                        else:
                            manipulating = log.additionalData[1]
                        if 'True' in log.additionalData[4]:
                            nearInteractionStartTime = log.time
                            manipulationType = "Near"
                        else:
                            farInteractionStartTime = log.time
                            manipulationType = "Far"
                    elif "ManipulationEnded" in log.functionName:
                            if manipulating is "":
                                print("Got manipulationEnded, but am not manipulating")
                            if log.additionalData[1] is manipulating:
                                manipulating = ""
                                if 'True' in log.additionalData[4]:
                                    nearInteractions.append(log.time - nearInteractionStartTime)
                                else:
                                    farInteractions.append(log.time - farInteractionStartTime)
                            else:
                                print("ended manipulation on something else than manipulation started on")
                            
                    elif "DeleteBox" in log.functionName:
                        if manipulating is not "":
                            if manipulationType is "Near":
                                nearInteractions.append(log.time - nearInteractionStartTime)
                            elif manipulationType is "Far":
                                farInteractions.append(log.time - farInteractionStartTime)
                            else:
                                print("unknown manipulation type?")

                    
                    

                taskNearInteractions.append(nearInteractions)
                taskFarInteractions.append(farInteractions)
            participantNearInteractions.append(taskNearInteractions)
            participantFarInteractions.append(taskFarInteractions)

        return participantNearInteractions, participantFarInteractions


def plotDurationOfInteractionsPerTask(trials: 'list[Trial]', useMedian=False):
    near, far = getDurationOfInteractions(trials)

    nearInteractions = [[],[],[]]
    farInteractions = [[],[],[]]
    for trial in near:
        for i, task in enumerate(trial):
            for interaction in task:
                nearInteractions[i].append(interaction)

    for trial in far:
        for i, task in enumerate(trial):
            for interaction in task:
                farInteractions[i].append(interaction) 

    if useMedian:
        near = [np.median(item) for item in nearInteractions]
        far = [np.median(item) for item in farInteractions]
    else:        
        near = [np.sum(item) for item in nearInteractions]
        far = [np.sum(item) for item in farInteractions]
        
    N = 3
    # menMeans = (20, 35, 30, 35, 27)
    # womenMeans = (25, 32, 34, 20, 25)

    width = 0.35
    ind = np.arange(N) + 1 +width/2.0 # the x locations for the groups
    ind2 = [x + width for x in ind]
    fig, ax = plt.subplots(1)
    #ax = fig.add_axes([0,0,1,1])
    ax.bar(ind, near, width, color='r')
    ax.bar(ind2, far, width, color='b')
    ax.set_ylabel('[s]')
    #ax.set_xlabel('Task')
    #ax.set_title('Scores by group and gender')
    ax.set_xticks([x + width/2.0 for x in ind]) #, labels=['Task 1', 'Task 2', 'Task 3'])
    ax.set_xticklabels(['Task 1', 'Task 2', 'Task 3'])
    # if not useMedian:
    #     ax.set_yticks(np.arange(0, 175, 20))
    ax.legend(labels=['Near interaction', 'Far interaction'])
    if useMedian:
        title = "Median duration of far/near interactions per task"
    else:
        title='Total duration of far/near interactions per task'
    fig.suptitle(title, fontsize=16)
    global logFolder
    if useMedian:
        fig.savefig(logFolder + '\\plotDurationOfInteractionsPerTaskMedian.pdf')
    else:
        fig.savefig(logFolder + '\\plotDurationOfInteractionsPerTask.pdf')


def checkSampleRate(trials: 'list[Trial]') -> None:
    # checking on sample rate,
    allSampletimes = [] 
    for trial in trials:
        sampleTimes = []
        for scene in trial.scenes:
            if "task_" in scene.name:
                for i in range(len(scene.positionLogs)):
                    if i == 0:
                        continue
                    sampleTimes.append(scene.positionLogs[i].time - scene.positionLogs[i-1].time)
        allSampletimes += sampleTimes
        # per trial sample times
        # startTime = trial.scenes[0].positionLogs[0].time
        # endTime = trial.scenes[-1].positionLogs[-1].time
        # duration = endTime-startTime
        # sampleRateAvg = np.average(sampleTimes)
        # sampleRateSTD = np.std(sampleTimes)
        # print(trial.folderName, 'Duration: ', duration, ', Sample rate mean: ', sampleRateAvg, ', Sample rate std.dev: ', sampleRateSTD)

    sampleRateAvgALL = np.average(np.array(allSampletimes))
    sampleRateSTDALL = np.std(np.array(allSampletimes))
    print(f"For all trials, sample rate mean: {sampleRateAvgALL}, sample rate std.dev: {sampleRateSTDALL}")


def updateAllPlots(trials: 'list[Trial]', show: bool = False) -> None:
    durationOfTasks = taskDurationStatistics(trials)
    plotDurationOfTasks(durationOfTasks)

    # plot total near/far interaction per user, as well as using median/average per task
    plotInteractionsPerUser(trials)
    plotInteractionsPerTask(trials)
    plotInteractionsPerTask(trials, useMedian=True)

    plotObjectsLookedAtPerTask(trials)

    # these two only works for 2D fixed ui scene, and not the shelf scene
    # this is due to how interactions start and end from the shelf, 
    # where both name and id are changed during the manipulation, making it impossible 
    # with currenty Unity implementation to check if it is the same box. 
    # plotDurationOfInteractionsPerTask(trials)
    # plotDurationOfInteractionsPerTask(trials, True)

    if show:
        plt.show()


if __name__ == '__main__':
    # first trials with the fixed UI:
    # logFolder = "C:\\Users\\ander\\Desktop\\final_logs_2d" 
    # test folder with improved shelf scene:
    global logFolder
    logFolder = "C:\\Users\\ander\\Desktop\\final_logs\\final_shelf_logs" 
    trials = importLogFolderToTrials(logFolder, prunePlayScene=True)
    trials = sorted(trials, key=lambda item: item.folderName)
    for trial in trials: trial.convertBoxNameToBoxType()

    calculateStatisticsOnTrials(trials)
    print(taskDurationStatistics(trials))
    updateAllPlots(trials, show=False)

    checkSampleRate(trials)

    