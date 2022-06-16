using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit;




//[AddComponentMenu("Scripts/MRTK/Examples/FollowEyeGaze")]

public class eye_gaze_test_script : MonoBehaviour
{
    [Tooltip("Display the game object along the eye gaze ray at a default distance (in meters).")]
    [SerializeField]
    private float defaultDistanceInMeters = 2f;
    private void Start()
    { 
    }
    void Update()
    {
        var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;

        if (eyeGazeProvider != null )
        {
            if(eyeGazeProvider.IsEyeTrackingEnabledAndValid)
            {
                gameObject.transform.position = eyeGazeProvider.HitPosition;
                var objectHitName = eyeGazeProvider.HitInfo.transform.gameObject.name;
                var objectHitPos = eyeGazeProvider.HitPosition;
                if (objectHitName == null)
                {
                    objectHitName = "Background";
                }
                else if (objectHitName.Contains("spatial"))
                {
                    objectHitName = "SpatialMesh";
                }
                if (objectHitPos == null)
                {
                    objectHitPos = Vector3.zero;
                }
            }
            
            /*
            EyeTrackingTarget lookedAtEyeTarget = EyeTrackingTarget.LookedAtEyeTarget;

            // Update GameObject to the current eye gaze position at a given distance
            if (lookedAtEyeTarget != null)
            {
                // Show the object at the center of the currently looked at target.
                if (lookedAtEyeTarget.EyeCursorSnapToTargetCenter)
                {
                    Ray rayToCenter = new Ray(CameraCache.Main.transform.position, lookedAtEyeTarget.transform.position - CameraCache.Main.transform.position);
                    RaycastHit hitInfo;
                    UnityEngine.Physics.Raycast(rayToCenter, out hitInfo);
                    gameObject.transform.position = hitInfo.point;
                }
                else
                {
                    // Show the object at the hit position of the user's eye gaze ray with the target.
                    gameObject.transform.position = eyeGazeProvider.GazeOrigin + eyeGazeProvider.GazeDirection.normalized * defaultDistanceInMeters;
                }
            }
            */
        }
    }
}
