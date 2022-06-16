using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class boxparam : MonoBehaviour
{
    public string type = "place";
    public List<int> ids;
    public TextMeshPro textMesh;
    RectTransform textTransform;
    public string text;
    GameObject mainCamera;
    GameObject motherOfBoxes;
    GameObject image_target1;
    Transform base_transform;
    Color original_color;
    public float distance;
    public float distance2;
    public GameObject[] texts;
    public GameObject overlay;
    public bool trashDropper = false;
    public GameObject deleteIn;
    GameObject shelf;
    public bool deleteFlag;

    // Start is called before the first frame update
    void Awake()
    {
        if (type == "None"){
            return;
                
        }

        deleteFlag = false;


    }
    void Start()
    {
        overlay.SetActive(false);
        textMesh = GetComponentInChildren<TextMeshPro>();
        textTransform = GetComponentInChildren<RectTransform>();
        mainCamera = GameObject.FindGameObjectsWithTag("MainCamera")[0];// assumes 1st camera is main camera
        motherOfBoxes = GameObject.Find("MotherOfBoxes");
        //image_target1 = GameObject.Find("ImageTarget1");
        shelf = GameObject.FindGameObjectWithTag("BoxHolder");
        base_transform = GameObject.FindGameObjectWithTag("Robot").transform;
        original_color = GetComponent<Renderer>().material.color;
        /*foreach(var text in texts)
        {
            text.GetComponent<TextMeshPro>().enabled = false;
        }*/

        textMesh.enabled = false;

        Vector2[] uvs = GetComponent<MeshFilter>().sharedMesh.uv; // turns material (icon) right direction everywhere

        uvs[6] = new Vector2(0, 0);
        uvs[7] = new Vector2(1, 0);
        uvs[10] = new Vector2(0, 1);
        uvs[11] = new Vector2(1, 1);

        GetComponent<MeshFilter>().sharedMesh.uv = uvs;

        boxText();
        deleteFlag = false;
    }
    // Update is called once per frame

    public void boxText()
    {
        
        for (int i = 0; i < texts.Length; i++)
        {
            texts[i].GetComponent<TextMeshPro>().text = getIdString();
            texts[i].transform.GetChild(0).gameObject.GetComponent<TextMeshPro>().text = type;
            //texts[i].GetComponentInChildren<TextMeshPro>().text = type;
        }

        /*for (int i = 0; i < texts.Length; i++)
        {
            texts[i].GetComponent<TextMeshPro>().text = "wtf";
        }*/
    }

    public void enabletext()
    {
        foreach (var text in texts)
            text.GetComponent<TextMeshPro>().enabled = true;
    }

    void rotateText()
    {
        if(textMesh.enabled)
        {
            //textMesh.text = "box type: " + type + "box id: " + id;
            textTransform.rotation = mainCamera.transform.rotation;
        }
    }

    void Update()
    {
        
        TrashDropper();
        Valid_placement();
        if(textMesh.enabled)
            rotateText();
    }

    void Valid_placement()
    {
        distance = Vector3.Distance(gameObject.transform.position, base_transform.position);
        distance2 = Vector2.Distance(new Vector2(gameObject.transform.position.x, gameObject.transform.position.z), new Vector2(base_transform.position.x, base_transform.position.z));


        float distanceInYToQRFromBaseLink = -.1f;
        if (gameObject.tag == "Box" && (gameObject.transform.localPosition.y - gameObject.transform.localScale.y/2.0f < distanceInYToQRFromBaseLink || distance > 1.3f || distance < 0.2f || distance2 < 0.15f)) // table height || max reach ws || min dex ws || singularity cylinder;
        {
            overlay.SetActive(true);
            OutOfBoundsIndicatorOn();
            //gameObject.GetComponent<Renderer>().material.color = new Color(255, 0, 0);
        }
        else
        {
            overlay.SetActive(false);
            OutOfBoundsIndicatorOff();
            //gameObject.GetComponent<Renderer>().material.color = original_color;
        }
    }
    public void OnHover()
    {
        //GetComponentInChildren<TextMeshPro>().enabled = true;
        DeleteIndicatorOn();
    }

    public void OnHoverExit()
    {
        //GetComponentInChildren<TextMeshPro>().enabled = false;
        DeleteIndicatorOff();
    }

    
    public void Delete(GameObject box)
    {
        if (motherOfBoxes.GetComponent<MotherOfBoxes>().deleteActive)
        {
            motherOfBoxes.GetComponent<MotherOfBoxes>().DeleteBox(box);
        }
    }

    public void DeleteIndicatorOn()
    {
        if (motherOfBoxes.GetComponent<MotherOfBoxes>().deleteActive && gameObject.tag == "Box")
        {
            deleteIn.gameObject.SetActive(true);
            textMesh.text = "Delete";
            textMesh.enabled = true;
        }
    }

    void OutOfBoundsIndicatorOn()
    {
        if (!motherOfBoxes.GetComponent<MotherOfBoxes>().deleteActive)
        {
            textMesh.text = "Outside of workspace";
            textMesh.enabled = true;
        }
    }

    void OutOfBoundsIndicatorOff()
    {
        if (!motherOfBoxes.GetComponent<MotherOfBoxes>().deleteActive)
        {
            textMesh.enabled = false;
        }


    }

    public void DeleteIndicatorOff()
    {
        deleteIn.gameObject.SetActive(false);
        textMesh.enabled = false;
        
    }
    void TrashDropper()
    {
        if (trashDropper)
        {
            gameObject.GetComponent<Rigidbody>().useGravity = true;
        }
        else
        {
            gameObject.GetComponent<Rigidbody>().useGravity = false;
        }
    }

    public void OnGrabEnter()
    {
        var BoxHolders = GameObject.FindGameObjectsWithTag("BoxHolder");
        foreach (var BoxHolder in BoxHolders)
        {
            BoxHolder.GetComponent<Collider>().enabled = true;
        }
    }

    public void OnGrabExit()
    {
        DisableCollision();
    }

    public void DisableCollision()
    {
        var BoxHolders = GameObject.FindGameObjectsWithTag("BoxHolder");
        foreach (var BoxHolder in BoxHolders)
        {
            BoxHolder.GetComponent<Collider>().enabled = false;
        }
    }

    public string getIdString()
    {
        if (ids.Count < 1) return "?";

        string idString = "";
        foreach (var id in ids)
        {
            idString += id.ToString() + ", ";
        }
        // trim trailing comma and space
        idString = idString.Remove(idString.Length - 2);
        return idString;
    }

    public void PlaceOnShelf()
    {
        if (shelf == null) return;

        if (deleteFlag)
        {
            motherOfBoxes.GetComponent<MotherOfBoxes>().DeleteBox(gameObject);
        }
        
}
}

