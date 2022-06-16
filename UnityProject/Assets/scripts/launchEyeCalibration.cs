using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class launchEyeCalibration : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string uri = "ms-hololenssetup://EyeTracking";
#if UNITY_WSA
        UnityEngine.WSA.Launcher.LaunchUri(uri, false);
#else
            Application.OpenURL(uri);
#endif
        /*
#if WINDOWS_UWP
    UnityEngine.WSA.Application.InvokeOnUIThread(async () =>
    {
        bool result = await global::Windows.System.Launcher.LaunchUriAsync(new System.Uri("ms-hololenssetup://EyeTracking"));
        if (!result)
        {
            Debug.LogError("Launching URI failed to launch.");
        }
    }, false);
#else
        Debug.LogError("Launching eye tracking not supported Windows UWP");
#endif
        */
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
