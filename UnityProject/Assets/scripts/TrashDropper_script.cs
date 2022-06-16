using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrashDropper_script : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Box")
        {
            other.gameObject.GetComponent<boxparam>().trashDropper = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.tag == "Box")
        {
            other.gameObject.GetComponent<boxparam>().trashDropper = false;

        }
    }
}