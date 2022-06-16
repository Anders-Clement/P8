using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class gripper_place_script : MonoBehaviour
{
    public float phaseMovement = 1.56f;
    public float phaseOffset = .0f;
    public float movementDuration = 3.0f;
    public float movementLength = .2f;

    // Start is called before the first frame update
    float startY;
    float startTime;
    float animationTime;
    
    void Start()
    {
        startY = transform.position.y;
        startTime = Time.time;
        animationTime = startTime;
    }

    // Update is called once per frame
    void Update()
    {
        animationTime += Time.deltaTime;
        float animationPos = ((animationTime - startTime) / movementDuration) * phaseMovement;
        Debug.Log("animationPos: " + animationPos.ToString());
        if (animationPos > phaseMovement) //animationTime > movementDuration + startTime ||
        {
            startTime = Time.time;
            animationTime = startTime;
            return;
        }
        float movement = Mathf.Sin(animationPos + phaseOffset)/phaseMovement*movementLength;
        var newPos = new Vector3(transform.position.x, startY + movement, transform.position.z);
        gameObject.transform.position = newPos;        
    }
}
