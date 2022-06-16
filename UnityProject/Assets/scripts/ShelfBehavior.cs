using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShelfBehavior : MonoBehaviour
{
    public GameObject MotherOfBoxes;
    Vector3 position;
    public GameObject PickPrefab;
    public GameObject PlacePrefab;
    public GameObject WaypointPrefab;
    public GameObject ShelfParent;
    

    // Start is called before the first frame update
    void Start()
    {
        gameObject.GetComponent<BoxCollider>().enabled = false;

    }

    public void SpawnStartBoxes()
    {
        Vector3 PlacePos = new Vector3(22f, 93.5f, 19.4285793f);
        Vector3 PickPos = new Vector3(-22f, 93.1999969f, 19.4285793f);
        Vector3 WaypointPos = new Vector3(22.0f, 128.5f, 19.4200001f);


        PickPrefab.transform.localScale = new Vector3(20f, 20f, 20f);
        WaypointPrefab.transform.localScale = new Vector3(20f, 20f, 20f);
        PlacePrefab.transform.localScale = new Vector3(20f, 20f, 20f);

        MotherOfBoxes.GetComponent<MotherOfBoxes>().SpawnBoxOnShelf(PickPrefab, PickPos, ShelfParent);
        MotherOfBoxes.GetComponent<MotherOfBoxes>().SpawnBoxOnShelf(PlacePrefab, PlacePos, ShelfParent);
        MotherOfBoxes.GetComponent<MotherOfBoxes>().SpawnBoxOnShelf(WaypointPrefab, WaypointPos, ShelfParent);

        PickPrefab.transform.localScale = new Vector3(.2f, .2f, .2f);
        WaypointPrefab.transform.localScale = new Vector3(.2f, .2f, .2f);
        PlacePrefab.transform.localScale = new Vector3(.2f, .2f, .2f);
    }
     
    private void OnTriggerExit(Collider other)
    {

        if (other.tag == "ShelfBox")
        {
            if (other.GetComponent<boxparam>().type == "place")
            {
                position = new Vector3(22f, 93.5f, 19.4285793f);
            }
            else if (other.GetComponent<boxparam>().type == "pick")
            {
                position = new Vector3(-22f, 93.1999969f, 19.4285793f);
            }
            else if (other.GetComponent<boxparam>().type == "waypoint")
            {
                position = new Vector3(22.0f, 128.5f, 19.4200001f);
            }
            else
            {
                position = new Vector3(0, 0, 0);
                Debug.Log("Put the box in default pos, because it was neither a place or pick obj");
            }
            MotherOfBoxes.GetComponent<MotherOfBoxes>().SpawnPlaceBoxOnShelf(other.gameObject, position, gameObject);
        }

        if(other.tag == "Box")
        {
            other.GetComponent<boxparam>().deleteFlag = false;
        }
       
       
    }
    private void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "Box")
        {
            other.gameObject.GetComponent<boxparam>().deleteFlag = true;
        }
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
