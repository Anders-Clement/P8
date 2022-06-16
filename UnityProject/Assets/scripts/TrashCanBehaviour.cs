using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrashCanBehaviour : MonoBehaviour
{
    public GameObject motherOfBoxes;
 
    // Start is called before the first frame update
    void Start()
    {
    
    }

    private void OnTriggerEnter(Collider other)
    {
        //Debug.Log("trigger");
        if(other.gameObject.tag == "Box")
            motherOfBoxes.GetComponent<MotherOfBoxes>().DeleteBox(other.gameObject);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
