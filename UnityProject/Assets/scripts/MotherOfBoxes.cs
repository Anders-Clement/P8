using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;
using RosSharp.RosBridgeClient;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SceneSystem;

public class MotherOfBoxes : MonoBehaviour
{
    public GameObject parent;
    public GameObject prefab;
    private BoxesPublisher publishComponent;
    private int id_counter = 0;
    private int total_num_boxes_spawned = 0;
    private UnityEngine.Events.UnityAction<ManipulationEventData> onManipulationEndedEvent, onManipulationStartedEvent;

    public string MainSceneName = "ManagerScene";
    private GameObject CameraGameObject;
    private GameObject base_link;
    public GameObject deleteButton;
    public GameObject sequencingButton;
    GameObject Rosbridge;
    public bool deleteActive = false;
    private bool sequencingActive = false;

    private holoutils.CSVLogger logger;

    private void tryLog(string logMsg)
    {
        if (logger != null)
            logger.AddRow(logMsg);
    }

    public GameObject SpawnBox(GameObject prefab)
    {
        // ensure deletetoggle is off, such that spawned box is not deleted immediately
        if (deleteActive)
        {
            disableDelete();
        }
        if (sequencingActive)
        {
            disableSequencing();
        }

        GameObject reftobox = Instantiate(prefab, CameraGameObject.transform.localPosition, prefab.GetComponent<Transform>().transform.rotation, CameraGameObject.GetComponent<Transform>());   // instansiate box
        boxparam refparam = reftobox.GetComponent<boxparam>();
        reftobox.transform.localPosition = new Vector3(0, 0, 1);
        reftobox.transform.SetParent(GetComponent<Transform>(), true);
        // check that box is not spawned under table (below the base_link)
        if(base_link != null)
        {
            var y_pos_in_base_link = base_link.transform.InverseTransformPoint(reftobox.transform.position).y;
            if (y_pos_in_base_link < 0)
            {
                reftobox.transform.position = new Vector3(reftobox.transform.position.x,
                                                            reftobox.transform.position.y + y_pos_in_base_link * -1.0f,
                                                            reftobox.transform.position.z);
            }
        }

        //reftobox.GetComponent<Transform>().localPosition = new Vector3(0, 0, 1);                            // give transform
        refparam.ids = new List<int> {id_counter};                                                         // set id
        reftobox.name = "Box " + (total_num_boxes_spawned++).ToString();
        refparam.enabletext();

        // refparam.type = box_type;                                                                        // set type
        id_counter += 1;

        ObjectManipulator refObjectmanipulator = reftobox.GetComponent<ObjectManipulator>();                // Get the object manipulator component of the box
        refObjectmanipulator.OnManipulationEnded.AddListener(onManipulationEndedEvent);                                     // add an event listener
        refObjectmanipulator.OnManipulationStarted.AddListener(onManipulationStartedEvent);
        
        publishComponent.SendMessage(reftobox);                           // put this on if we need to publish info on box when it initializes (before it is moved)
        
        return reftobox;

    }

   

    /*public void OnHover(GameObject box)
    {

        box.GetComponentInChildren<TextMeshPro>().enabled = true;

        //Debug.Log("hovering");
    }*/

    /*public void OnHoverExit(GameObject box)
    {
        box.GetComponentInChildren<TextMeshPro>().enabled = false;
    }*/

    public void SpawnPlaceBox(GameObject placePrefab)
    {
        SpawnBox(placePrefab);
        tryLog("FUNCTION;SpawnPlaceBox");
    }

    public void SpawnPickupBox(GameObject pickPrefab)
    {
        SpawnBox(pickPrefab);
        tryLog("FUNCTION;SpawnPickUpBox");
    }

    public void SpawnWP(GameObject wpPrefab)
    {
        SpawnBox(wpPrefab);
        tryLog("FUNCTION;SpawnWP");
    }

    public void SpawnBoxOnShelf(GameObject TakenplacePrefab, Vector3 position, GameObject parent)
    {
        // instansiate box on shelf, same type, id = -1
        GameObject reftobox = Instantiate(TakenplacePrefab, parent.GetComponent<Transform>().localPosition, parent.GetComponent<Transform>().transform.rotation, parent.GetComponent<Transform>());   // instansiate box
        boxparam refparam = reftobox.GetComponent<boxparam>();
        reftobox.tag = "ShelfBox";

        ObjectManipulator refObjectmanipulator = reftobox.GetComponent<ObjectManipulator>();                // Get the object manipulator component of the box
        refObjectmanipulator.OnManipulationEnded.AddListener(onManipulationEndedEvent);                                             // add an event listener
        refObjectmanipulator.OnManipulationStarted.AddListener(onManipulationStartedEvent);

        reftobox.name = TakenplacePrefab.name;
        // set the position given type

        reftobox.transform.localPosition = position;

        refparam.ids[0] = -1;
    }

    public void SpawnPlaceBoxOnShelf(GameObject TakenplacePrefab, Vector3 position, GameObject parent)
    {
        tryLog("FUNCTION;SpawnBoxOnShelf;" + TakenplacePrefab.GetComponent<boxparam>().type);

        SpawnBoxOnShelf(TakenplacePrefab, position, parent);

        // make the taken box the next id
        TakenplacePrefab.GetComponent<boxparam>().ids[0] = id_counter++;
        // give the Box tag
        TakenplacePrefab.tag = "Box";
        // set the parent to MotherOfBoxes
        TakenplacePrefab.GetComponent<boxparam>().enabletext();

        TakenplacePrefab.transform.SetParent(GetComponent<Transform>(), true);

        //TakenplacePrefab.transform.parent = gameObject.transform;
        
        // Change name in unity
        TakenplacePrefab.name = "Box " + (total_num_boxes_spawned++).ToString();
        // Update box text
        TakenplacePrefab.GetComponent<boxparam>().boxText();

        
        
    }

    public void DeleteToggle()
    {
        if (sequencingActive)
        {
            disableSequencing();
        }

        deleteActive = !deleteActive;
        setButtonBackPlateButton(deleteButton, deleteActive);
        tryLog("FUNCTION;DeleteToggle;" + deleteActive.ToString());
        foreach (Transform child in gameObject.GetComponentInChildren<Transform>())
        {
            if (deleteActive)
            {
                child.gameObject.GetComponent<boxparam>().DeleteIndicatorOn();
            }
            else if (!deleteActive)
            {
                child.gameObject.GetComponent<boxparam>().DeleteIndicatorOff();
            }
        }
    }

    private void disableDelete()
    {
        if (deleteActive)
        {
            deleteActive = false;
            //deleteButton.GetComponent<PhysicalPressEventRouter>().OnHandPressTriggered();
            tryLog("FUNCTION;DeleteToggle;" + deleteActive.ToString());
        }
        setButtonBackPlateButton(deleteButton, false);
        foreach (Transform child in gameObject.GetComponentInChildren<Transform>())
        {
                child.gameObject.GetComponent<boxparam>().DeleteIndicatorOff();
        }
    }


    private void DeleteBoxNoLog(GameObject box_to_delete)
    {
        if (box_to_delete.tag == "Box")
        {
            // ids must be popped in descending order
            var ids = box_to_delete.GetComponent<boxparam>().ids;
            ids.Sort();
            for(int n = ids.Count-1; n >= 0; n--)
            {
                int id = ids[n];
                for (int i = 0; i < parent.transform.childCount; i++)
                {
                    GameObject child = parent.transform.GetChild(i).gameObject;
                    if (box_to_delete.gameObject.Equals(child)) continue;

                    var childIds = child.GetComponent<boxparam>().ids;
                    for (int j = 0; j < childIds.Count; j++)
                    {
                        if (childIds[j] > id)
                        {
                            childIds[j] -= 1;
                            // ids are now kept unique on the gameobjects as names
                            // child.name = "Box " + (child.GetComponent<boxparam>().id).ToString();
                            child.GetComponent<boxparam>().boxText();
                        }
                    }   
                }
                id_counter -= 1;
            }

            box_to_delete.GetComponent<boxparam>().type = "delete";
            // sort ids by descending for delete, so it works on ROS side
            ids.Sort((a, b) => b.CompareTo(a));
            publishComponent.SendMessage(box_to_delete);
            Destroy(box_to_delete);
            //Destroy(box_to_delete, 5);
        }
    }

    public void DeleteBox(GameObject box_to_delete)
    {
        if (sequencingActive) disableSequencing();

        box_to_delete.GetComponent<boxparam>().DisableCollision();
        tryLog("FUNCTION;DeleteBox;" + box_to_delete.GetComponent<boxparam>().getIdString() + ";" + box_to_delete.name);
        DeleteBoxNoLog(box_to_delete);
    }

    public void DeleteAll()
    {
        if (sequencingActive) disableSequencing();
        if (deleteActive) disableDelete();

        tryLog("FUNCTION;DeleteAll;");
        for (int i = parent.transform.childCount-1; i >= 0; i--)
        {
            GameObject child = parent.transform.GetChild(i).gameObject;
            DeleteBoxNoLog(child);
        }
    }

        
  
    public void Sequencing(boxparam param) // 
    {
        param.ids.Add(id_counter++);
        param.boxText();
    }
    void manipulation_started(ManipulationEventData data)
    {
        if (sequencingActive)
        {
            var boxparam = data.ManipulationSource.GetComponent<boxparam>();
            if (boxparam == null) return;
            Sequencing(boxparam);
        }
        else
        {
            GameObject refToMovedBox = data.ManipulationSource;
            // Debug.Log("near or far interaction: " + data.IsNearInteraction.ToString());
            tryLog("FUNCTION;ManipulationStarted;" +
                    refToMovedBox.GetComponent<boxparam>().getIdString() +
                    ";" + refToMovedBox.name + ";" +
                    base_link.transform.InverseTransformPoint(refToMovedBox.transform.position).ToString("F7") + ";" +
                    "IsNearInteraction;" + data.IsNearInteraction.ToString());
        }
    }


    void Mainipulation_ended(ManipulationEventData data)                                                    //Function called when manipulation ends
    {
        if (sequencingActive) return;

        Rosbridge = GameObject.Find("Rosbridge");
        publishComponent = Rosbridge.GetComponent<BoxesPublisher>();
        GameObject refToMovedBox = data.ManipulationSource;

        tryLog("FUNCTION;ManipulationEnded;" + 
            refToMovedBox.GetComponent<boxparam>().getIdString() +
            ";" + refToMovedBox.name + ";" +
            base_link.transform.InverseTransformPoint(refToMovedBox.transform.position).ToString("F7") + ";" +
            "IsNearInteraction;" + data.IsNearInteraction.ToString());

        // legacy code for avoiding publishing boxes moved within the crate/shelf
        if (refToMovedBox.GetComponent<boxparam>().ids[0] == -1)
        {
            return;
        }
        publishComponent.SendMessage(refToMovedBox);                                                        //Publish pose to ROS
    }
   
    void Start()
    {
        
        Rosbridge = GameObject.Find("Rosbridge");
        publishComponent = Rosbridge.GetComponent<BoxesPublisher>();

        onManipulationEndedEvent += Mainipulation_ended;
        onManipulationStartedEvent += manipulation_started;
        prefab.GetComponent<boxparam>().DisableCollision();

        //SpawnPlaceBox();                                                                     // Spawn boxes 
        //SpawnBox();

        /*
        var sceneSystem = MixedRealityToolkit.Instance.GetService<IMixedRealitySceneSystem>();
        var objects = sceneSystem.GetScene(MainSceneName).GetRootGameObjects();
        foreach (GameObject test_obj in objects) {
            if (test_obj.name == "MixedRealityPlayspace")
            {
                if (test_obj.transform.Find("Main Camera") != null)
                {
                    CameraGameObject = test_obj.transform.Find("Main Camera").gameObject;
                    Debug.Log("Found the GameObject of the camera");
                    break;
                };
            }
        }*/
        var cams = GameObject.FindGameObjectsWithTag("MainCamera");
        if (cams.Length != 1)
        {
            Debug.LogError("Not 1 main camera, got " + cams.Length.ToString() + " cameras. Using first camera.");
        }
        CameraGameObject = cams[0];

        var loggers = GameObject.FindGameObjectsWithTag("Logger");
        if (loggers.Length != 1)
        {
            Debug.LogError("Not 1 logger, got " + loggers.Length.ToString() + " loggers. Using first logger.");
        }
        logger = loggers[0].GetComponent<holoutils.CSVLogger>();

        var robots = GameObject.FindGameObjectsWithTag("Robot");
        if (robots.Length > 1)
        {
            Debug.LogError("Not 1 robot got " + robots.Length.ToString() + " robots. Using first robot.");
            foreach (var name in robots)
            {
                Debug.Log(name.name);
            }
            base_link = robots[0];
        }
        else if (robots.Length == 1)
            base_link = robots[0];
    }

    private void setButtonBackPlateButton(GameObject button, bool value)
    {
        if (button == null) return;
        for (int i = 0; i < button.transform.childCount; i++)
        {
            var child = button.transform.GetChild(i);
            if (child.name == "BackPlateToggleState")
            {
                child.gameObject.SetActive(value);
                break;
            }
        }
    }

    public void disableSequencing() // diable on grab end 
    {
        if(sequencingActive)
        {
            sequencingActive = false;
            //sequencingButton.GetComponent<PhysicalPressEventRouter>().OnHandPressTriggered();
            Debug.Log("MoB: disableSequencing to " + sequencingActive.ToString());
        }
        setButtonBackPlateButton(sequencingButton ,false);
        for (int i = 0; i < parent.transform.childCount; i++)
        {
            GameObject child = parent.transform.GetChild(i).gameObject;
            //child.GetComponent<Rigidbody>().isKinematic = false;
            child.GetComponent<MoveAxisConstraint>().ConstraintOnMovement = 0;
            child.GetComponent<RotationAxisConstraint>().ConstraintOnRotation = Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.XAxis | Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.ZAxis;
            var childBoxParam = child.GetComponent<boxparam>();
            if (childBoxParam.ids.Count == 0)
            {
                childBoxParam.ids.Add(id_counter++);
                childBoxParam.boxText();
            }
            publishComponent.SendMessage(child);                                                        //Publish pose to ROS
        }
    }

    public void enableSequencing() // enalbe on grab start
    {
        Debug.Log("start seq");
        if(deleteActive)
        {
            disableDelete();
        }
        if(!sequencingActive)
        {
            sequencingActive = true;
            Debug.Log("MoB: enableSequencing to " + sequencingActive.ToString());
        }
        setButtonBackPlateButton(sequencingButton ,true);
        id_counter = 0;
        for (int i = 0; i < parent.transform.childCount; i++)
        {
            GameObject child = parent.transform.GetChild(i).gameObject;
            //child.GetComponent<Rigidbody>().isKinematic = true;
            child.GetComponent<MoveAxisConstraint>().ConstraintOnMovement = Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.XAxis | Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.YAxis | Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.ZAxis;
            child.GetComponent<RotationAxisConstraint>().ConstraintOnRotation = Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.XAxis | Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.YAxis | Microsoft.MixedReality.Toolkit.Utilities.AxisFlags.ZAxis;
            var childBoxParam = child.GetComponent<boxparam>();

            // send delete message to ROS, to clean up that side
            var type = childBoxParam.type;
            childBoxParam.type = "delete";
            var numIds = childBoxParam.ids.Count;
            childBoxParam.ids.Clear();
            childBoxParam.ids.Add(0);
            for (int j = 0; j < numIds; j++)
            {
                publishComponent.SendMessage(child);
            }

            // restore type of box
            childBoxParam.type = type;
            childBoxParam.ids.Clear();
            childBoxParam.boxText();
        }
    }

    public void toggleSequencing()
    {
        sequencingActive = !sequencingActive;
        Debug.Log("MoB: toggleSequencing to " + sequencingActive.ToString());
        if(sequencingActive)
        {
            enableSequencing();
        }
        else
        {
            disableSequencing();
        }
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
