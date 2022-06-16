using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Microsoft.MixedReality.Toolkit.SceneSystem;
using Microsoft.MixedReality.Toolkit;


public class SceneChangeButtonScript : MonoBehaviour
{
    public GameObject loadingIcon = null;
    SceneChangesub scenesubber;
    int currentScene;
    string[] sceneNames;
    Microsoft.MixedReality.Toolkit.SceneSystem.IMixedRealitySceneSystem sceneSystem;
    // Start is called before the first frame update
    void Start()
    {

        scenesubber = gameObject.transform.parent.GetComponent<SceneChangesub>();
        if (scenesubber == null)
            Debug.Log("SceneChangeButtonScript could not get scenechangesub component!");

        sceneSystem = MixedRealityToolkit.Instance.GetService<IMixedRealitySceneSystem>();
        sceneNames = sceneSystem.ContentSceneNames;
        currentScene = 0;
    }

    async void SendMessage(string uselessString)
    {
        if (loadingIcon != null)
            loadingIcon.SetActive(true);

        currentScene++;
        if (currentScene >= sceneNames.Length)
            currentScene = 0;
        var sceneName = sceneNames[currentScene];
        foreach (var scene in sceneNames)
        {
            if (sceneSystem.IsContentLoaded(scene) && scene != sceneName)
                await sceneSystem.UnloadContent(scene);
        }

        Debug.Log("Loading " + sceneName);
        await sceneSystem.LoadContent(sceneName);
        if (loadingIcon != null)
            loadingIcon.SetActive(false);
    }
}
