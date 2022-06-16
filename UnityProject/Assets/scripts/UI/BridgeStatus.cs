using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;


public class BridgeStatus : MonoBehaviour
{
    public TextMeshPro statusUI;
    public TextMeshPro IPText;
    public string msg;
    public GameObject rosBridge;
    private RosSharp.RosBridgeClient.RosConnector rosBridgeScript;
    public GameObject canvas;
    //private RosSharp.RosBridgeClient.RosBridgePingSub pingSub;
    TouchScreenKeyboard keyboard;
    public static string keyboardText = "";


    // Start is called before the first frame update
    void Start()
    {
        rosBridge = GameObject.Find("Rosbridge");
        rosBridgeScript = rosBridge.GetComponent<RosSharp.RosBridgeClient.RosConnector>();
        //pingSub = rosBridge.GetComponent<RosSharp.RosBridgeClient.RosBridgePingSub>();
        msg = rosBridgeScript.RosStatus;
        msg = "waiting for connection";
        IPText.text = rosBridgeScript.RosBridgeServerUrl;
        
    }

    void setStatus()
    {

        msg = rosBridgeScript.RosStatus;

        if (keyboard != null)
            rosBridgeScript.RosBridgeServerUrl = keyboard.text;
        IPText.text = rosBridgeScript.RosBridgeServerUrl;
        
        /*if (pingSub.lostConnection && rosBridgeScript.connected)
        {
            msg = "Disconnected from RosBridge (Have you remembered to launch or disable ping?)";
        }*/
        statusUI.SetText(msg);
    }


    public void SetIP()
    {
        
        keyboard = TouchScreenKeyboard.Open(rosBridgeScript.RosBridgeServerUrl, TouchScreenKeyboardType.Default);

        //rosBridgeScript.RosBridgeServerUrl = keyboard.text;
        //IPText.text = rosBridgeScript.RosBridgeServerUrl;


    }
    // Update is called once per frame
    void Update()
    {
        
        setStatus();
    }

    public void CloseUI()
    {
        canvas.SetActive(false);
    }

    public void reconnectToROS()
    {
        ROSreset();        
    }

    void ROSreset()
    {
        rosBridge = GameObject.Find("Rosbridge");

        
        Instantiate(rosBridge, new Vector3(0, 0, 0), Quaternion.identity);
        Destroy(rosBridge);
        
        rosBridge = GameObject.Find("Rosbridge(Clone)");
        rosBridge.name = "Rosbridge";
        rosBridgeScript = rosBridge.GetComponent<RosSharp.RosBridgeClient.RosConnector>();
        //pingSub = rosBridge.GetComponent<RosSharp.RosBridgeClient.RosBridgePingSub>();
    }

}
