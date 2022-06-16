using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;



public class SphereSequencingBehavior : MonoBehaviour
{
    public MotherOfBoxes motherofboxes;
    Vector3 startPosition;
    public float coolDown = 2f;
    public TextMeshPro textMesh;
    Camera mainCamera;
    public float distance = 0.3f;
    bool distFlag = false;
    bool seqFlag = false;
    bool grabbed = false;
    float timeWhenLetGo;
    // Start is called before the first frame update
    void Start()
    {
        startPosition = gameObject.transform.localPosition;
        distFlag = false;
        var cams = GameObject.FindGameObjectsWithTag("MainCamera");
        if (cams.Length != 1)
        {
            Debug.LogError("Not 1 main camera, got " + cams.Length.ToString() + " cameras. Using first camera.");
        }
        mainCamera = cams[0].GetComponent<Camera>();
        seqFlag = false;
        grabbed = false;
    }


    // Update is called once per frame
    void Update()
    {
        if(!distFlag && Vector3.Distance(gameObject.transform.localPosition, startPosition) > distance){
            distFlag = true;
        }
        rotateText();
        if(Time.time - timeWhenLetGo > coolDown && !grabbed && distFlag)
        {
            returnSequencer();
           
        }
    }

    public void OnGrab()
    {
        //distFlag = false;
        grabbed = true;
        if (!seqFlag)
        {
            motherofboxes.enableSequencing();
            seqFlag = true;
        }
    }

    public void onNotGrab()
    {
        
        if (distFlag && Vector3.Distance(gameObject.transform.localPosition, startPosition) < distance )
        {
            returnSequencer();
        }
        timeWhenLetGo = Time.time;
        grabbed = false;
    }
    void rotateText()
    {
        var textRotation = textMesh.transform.rotation;
        textRotation = mainCamera.transform.rotation;
        textMesh.transform.rotation = textRotation;
    }
    
    void returnSequencer()
    {

        gameObject.transform.localPosition = startPosition;
        motherofboxes.disableSequencing();
        seqFlag = false;
        distFlag = false;
        Debug.Log("Returning sequencer");
    }
    IEnumerator returnToStart()
    {
        yield return new WaitForSeconds(coolDown);
        Debug.Log("return");
        returnSequencer();
        //gameObject.transform.position = new Vector3(0, 0, 0);
        
    }


    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag == "Box" && grabbed)
        {
            motherofboxes.Sequencing(other.gameObject.GetComponent<boxparam>());
        }
    }

}
