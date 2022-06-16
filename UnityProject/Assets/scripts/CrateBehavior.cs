using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class CrateBehavior : MonoBehaviour
{
    public GameObject MotherOfBoxes;
    Vector3 position;
    TextMeshPro textMesh;
    private Camera mainCamera;
    // Start is called before the first frame update
    void Start()
    {
        textMesh = GetComponentInChildren<TextMeshPro>();
        var cams = GameObject.FindGameObjectsWithTag("MainCamera");
        if (cams.Length != 1)
        {
            Debug.LogError("Not 1 main camera, got " + cams.Length.ToString() + " cameras. Using first camera.");
        }
        mainCamera = cams[0].GetComponent<Camera>();
    }

    private void OnTriggerExit(Collider other)
    {

        if (other.tag == "ShelfBox")
        {
            
            if (other.GetComponent<boxparam>().type == "place")
            {
                if (other.name == "crateBoxL") {

                    position = new Vector3(-1.31570017f, .0f, -0.0375000238f);
                }
                else if(other.name == "crateBoxR")
                {
                    position = new Vector3(-1.23800004f, .0f, 0.211799979f);
                }
                
            }
            else if (other.GetComponent<boxparam>().type == "pick")
            {
                if (other.name == "crateBoxL")
                {
                    position = new Vector3(-1.28090012f, -0.00839991868f, 0.627499938f);
                }
                else if (other.name == "crateBoxR")
                {
                    position = new Vector3(-1.32350004f, 0.0f, .89f);
                }
            }
            else
            {
                position = new Vector3(0, 0, 0);
                Debug.Log("Put the box in default pos, because it was neither a place or pick obj");
            }
            MotherOfBoxes.GetComponent<MotherOfBoxes>().SpawnPlaceBoxOnShelf(other.gameObject, position, transform.parent.gameObject);
            //Debug.Log(position);
        }


    }

    void rotateText()
    {
        var textRotation = textMesh.transform.rotation;
        textRotation = mainCamera.transform.rotation;
        textMesh.transform.rotation = textRotation;
    }

    // Update is called once per frame
    void Update()
    {
        rotateText();
    }
}
