using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using UnityEngine.SceneManagement;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SceneSystem;
using holoutils;
public class CheckForSceneChanger : MonoBehaviour
{
    SceneChangesub scenesubber;
    public GameObject loadingIcon = null;
    // Start is called before the first frame update
    void Start()
    {
        scenesubber = gameObject.GetComponent<SceneChangesub>();
    }

    // Update is called once per frame
    async void Update()
    {
        if (scenesubber.messageData != -1)
        {
            var sceneSystem = MixedRealityToolkit.Instance.GetService<IMixedRealitySceneSystem>();
            var sceneNames = sceneSystem.ContentSceneNames;

            int scene_to_load = scenesubber.messageData;
            scenesubber.messageData = -1;

            if (scene_to_load > sceneNames.Length)
            {
                Debug.Log("ERROR;CheckForSceneChanger: scene_to_load > sceneNames.Length == true");
                return;
            }
            var sceneName = sceneNames[scene_to_load];
            // Debug.Log(SceneManager.sceneCountInBuildSettings);
            //Debug.Log("Wtf");


            /*Scene currentScene = SceneManager.GetActiveScene();
            string sceneName = currentScene.name;
            Debug.Log(sceneName);*/
            if (loadingIcon != null)
                loadingIcon.SetActive(true);
            foreach(var scene in sceneNames)
            {
                if (sceneSystem.IsContentLoaded(scene) && scene != sceneName)
                    await sceneSystem.UnloadContent(scene);
            }

            Debug.Log("Loading " + sceneName);
            await sceneSystem.LoadContent(sceneName);
            if (loadingIcon != null)
                loadingIcon.SetActive(false);

            // Debug.Log("Setting Active");
            // SceneManager.SetActiveScene(SceneManager.GetSceneByBuildIndex(scene_to_load));
        }
    }
}
