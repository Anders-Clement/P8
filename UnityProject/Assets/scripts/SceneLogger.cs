using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneLogger : MonoBehaviour
{

    void test()
    {
        var scenes = SceneManager.GetAllScenes();
        foreach (var scene in scenes)
        {
            Debug.Log(scene.name);
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        Invoke("test", 2.0f);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
