using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Vuforia;

public class StopVuforia : MonoBehaviour
{
    public GameObject all_elements;
    public GameObject parent_after_tracking;
    public AudioSource audio = null;
    private holoutils.CSVLogger logger;
    GameObject shelf;

    void stopVuforia()
    {
        var stateManager = TrackerManager.Instance.GetStateManager();
        var behaviors = stateManager.GetTrackableBehaviours();
        foreach (var behavior in behaviors)
        {
            if (behavior != null)
            {
                if(behavior.name == "ImageTarget1")
                {
                    var position = all_elements.transform.parent.transform.TransformPoint(all_elements.transform.localPosition);
                    all_elements.transform.parent = parent_after_tracking.transform;
                    all_elements.transform.localPosition = parent_after_tracking.transform.InverseTransformPoint(position);
                    behavior.enabled = false;
                    
                    shelf.GetComponent<Collider>().enabled = false;
                    Debug.Log("Stopped " + behavior.ToString() + ", name: " + behavior.name);
                }
            }
        }
    }

    // bogus function name,  fix to be able to call it as event from vuforia when image target is found
    void SendMessage(string data)
    {
        Invoke("stopVuforia", 3.0f);
        shelf.GetComponent<ShelfBehavior>().SpawnStartBoxes();
        if (logger != null)
        {
            logger.doSceneStart();
        }
        if (audio != null)
            audio.enabled = true;
        /*
        var comps = mainCamera.GetComponents(typeof(Component));
        foreach(var comp in comps)
        {
            Debug.Log(comp.name + ", " + comp.GetType().ToString());
        }
        var instance = VuforiaBehaviour.Instance;
        
        if(instance != null)
        {
            if (instance.SetMaximumSimultaneousTrackedImages(0))
            {
                Debug.Log("SetMaximumSimultaneousTrackedImages = 0 success");
            }
            else
            {
                Debug.Log("SetMaximumSimultaneousTrackedImages = 0 failed");
            }
        }
        else
        {
            Debug.Log("VuforiaBehavior.instance == null");
        }
        */
    }

    // Start is called before the first frame update
    void Start()
    {
        shelf = GameObject.FindGameObjectWithTag("BoxHolder");
        audio = transform.GetComponentInParent<AudioSource>();
        var loggers = GameObject.FindGameObjectsWithTag("Logger");
        if (loggers.Length != 1)
        {
            Debug.LogError("Not 1 logger, got " + loggers.Length.ToString() + " loggers. Using first logger.");
        }
        logger = loggers[0].GetComponent<holoutils.CSVLogger>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
