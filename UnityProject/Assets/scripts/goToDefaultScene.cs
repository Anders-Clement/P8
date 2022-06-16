using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;


public class goToDefaultScene : MonoBehaviour
{
    public string defaultScene;
    // Start is called before the first frame update
    void Start()
    {
        SceneManager.LoadScene(defaultScene);
    }
}
