using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIManager : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject target;
    GameObject CameraGameObject;
    void Start()
    {
        CameraGameObject = GameObject.FindGameObjectWithTag("MainCamera");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ToggleElement(GameObject target)
    {
        if (target.activeSelf)
        {
            target.SetActive(false);
        }
        else
        {
            target.SetActive(true);
        }
    }

    public void ReCenter()
    {
        //gameObject.transform.position = CameraGameObject.transform.position + new Vector3(-1f,0,0);
        gameObject.transform.rotation = CameraGameObject.transform.rotation;
        gameObject.transform.position = Camera.main.transform.position + Camera.main.transform.forward * .5f + Camera.main.transform.right * -0.15f * gameObject.transform.localScale.x;
        // var parent = gameObject.transform.parent;
        // gameObject.transform.parent = CameraGameObject.transform;
        // gameObject.transform.localPosition = new Vector3(.0f, .0f, 1.0f);
        // gameObject.transform.position = CameraGameObject.transform.position + CameraGameObject.transform.TransformPoint(new Vector3(0f, 0f, 1f));
        // gameObject.transform.rotation = CameraGameObject.transform.rotation;
        // gameObject.transform.SetParent(parent, true);

    }

}
