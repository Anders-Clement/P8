using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.SceneManagement;
using Microsoft.MixedReality.Toolkit;
using RosSharp.RosBridgeClient;

#if WINDOWS_UWP
using Windows.Storage;
#endif

/*
 *  Based on following work: https://gist.github.com/julenka/20ece5141e821cb6d0a9cb530d3dc96d
 */

namespace holoutils
{
    /// <summary>
    /// Component that Logs data to a CSV.
    /// Assumes header is fixed.
    /// Copy and paste this logger to create your own CSV logger.
    /// CSV Logger breaks data up into settions (starts when application starts) which are folders
    /// and instances which are files
    /// A session starts when the application starts, it ends when the session ends.
    /// 
    /// In Editor, writes to MyDocuments/SessionFolderRoot folder
    /// On Device, saves data in the Pictures/SessionFolderRoot
    /// 
    /// How to use:
    /// Find the csvlogger
    /// if it has not started a CSV, create one.
    /// every frame, log stuff
    /// Flush data regularly
    /// 
    /// **Important: Requires the PicturesLibrary capability!**
    /// </summary>
    public class CSVLogger : MonoBehaviour

    {
        #region Constants to modify
        private const string DataSuffix = "data";
        private const string CSVHeader = "Timestamp;hololensPos;HololensOrientation(euler);eyeGazeObject;eyeGazePos";
        private const string SessionFolderRoot = "CSVLogger";
        private const string seperator = ";";
        #endregion

        #region private members
        private string m_sessionPath;
        private string m_filePath;
        private string m_recordingId;
        private string m_sessionId;
        private GameObject mainCamera;
        private GameObject base_link;
        private StringBuilder m_csvData;
        private float timeSinceLastLog;
        private int numLogs;
        private StringPublisher experimentStatusPub;

        #endregion
        #region public members
        public string RecordingInstance => m_recordingId;
        public float logInterval = .1f;
        public bool doEnd = false;
        #endregion

        // Use this for initialization
        async void Start()
        {
            // ready loggger
            await MakeNewSession();
            StartNewCSV();
            // AddRow("test");
            FlushData();

            getTransforms();
            timeSinceLastLog = 0;
            numLogs = 0;
            // flush data to csv file once per second
            //InvokeRepeating("FlushData", 1, 1);

            var rosBridgeManager = GameObject.FindGameObjectWithTag("RosBridgeManager");
            experimentStatusPub = rosBridgeManager.GetComponent<StringPublisher>();
            experimentStatusPub.publishMessage("Logger started");
        }

        private void Update()
        {
            timeSinceLastLog += Time.deltaTime;
            if(timeSinceLastLog > logInterval)
            {
                numLogs++;
                // add leftover time, so we do not log every ~.1x secs, instead of every .1 secs
                timeSinceLastLog = timeSinceLastLog - logInterval;
                logData();
                // Disabled, since flushing freeses hololens for a sec
                /*
                if (numLogs % 100 == 0)
                    FlushData();
                */
            }
            if (doEnd)
            {
                doEnd = false;
                doSceneEnd();
            }
        }

        async Task MakeNewSession()
        {
            m_sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string rootPath = "";
#if WINDOWS_UWP
            StorageFolder sessionParentFolder = await KnownFolders.PicturesLibrary
                .CreateFolderAsync(SessionFolderRoot,
                CreationCollisionOption.OpenIfExists);
            rootPath = sessionParentFolder.Path;
#else
            rootPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), SessionFolderRoot);
            if (!Directory.Exists(rootPath)) Directory.CreateDirectory(rootPath);
#endif
            m_sessionPath = Path.Combine(rootPath, m_sessionId);
            Directory.CreateDirectory(m_sessionPath);
            Debug.Log("CSVLogger logging data to " + m_sessionPath);
        }

        public void StartNewCSV()
        {
            m_recordingId = DateTime.Now.ToString("yyyyMMdd_HHmmssfff");
            var filename = m_recordingId + "-" + DataSuffix + ".csv";
            m_filePath = Path.Combine(m_sessionPath, filename);
            if (m_csvData != null)
            {
                EndCSV();
            }
            m_csvData = new StringBuilder();
            m_csvData.AppendLine(CSVHeader);
        }


        public void EndCSV()
        {
            if (m_csvData == null)
            {
                return;
            }
            using (var csvWriter = new StreamWriter(m_filePath, true))
            {
                csvWriter.Write(m_csvData.ToString());
            }
            m_recordingId = null;
            m_csvData = null;
        }

        public void OnDestroy()
        {
            EndCSV();
        }

        public void AddRow(List<String> rowData)
        {
            AddRow(string.Join(",", rowData.ToArray()));
        }

        public void AddRow(string row)
        {
            m_csvData.AppendLine(Time.realtimeSinceStartup.ToString("0.000") + seperator + row);
        }

        /// <summary>
        /// Writes all current data to current file
        /// </summary>
        public void FlushData()
        {

            using (var csvWriter = new StreamWriter(m_filePath, true))
            {
                csvWriter.Write(m_csvData.ToString());
            }

            m_csvData.Clear();

            Debug.Log("Flushing data to CSV");
        }

        private void Awake()
        {
            DontDestroyOnLoad(this.gameObject);
            SceneManager.sceneLoaded += onSceneLoaded;
        }

        void onSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            getTransforms();
            AddRow("FUNCTION;SceneChangeTo;" + scene.name);
            FlushData();
        }

        void getTransforms()
        {
            // get objects, so we can transform to base_link for hololens position
            if(mainCamera == null)
            {
                var cams = GameObject.FindGameObjectsWithTag("MainCamera");
                if (cams.Length != 1)
                {
                    Debug.LogError("Not 1 main camera, got " + cams.Length.ToString() + " cameras. Using first camera.");
                }
                mainCamera = cams[0];
            }
            
            if(base_link == null)
            {
                var robots = GameObject.FindGameObjectsWithTag("Robot");
                if (robots.Length > 1)
                {
                    Debug.LogError("Not 1 robot got " + robots.Length.ToString() + " robots. Using first robot.");
                    foreach (var name in robots)
                    {
                        Debug.Log(name.name);
                    }
                    base_link = robots[0];
                }
                else if (robots.Length == 1)
                    base_link = robots[0];
            }
        }

        /*
        /// <summary>
        /// Returns a row populated with common start data like
        /// recording id, session id, timestamp
        /// </summary>
        /// <returns></returns>
        public List<String> RowWithStartData()
        {
            List<String> rowData = new List<String>();
            rowData.Add(Time.timeSinceLevelLoad.ToString("##.000"));
            rowData.Add(m_recordingId);
            rowData.Add(m_recordingId);
            return rowData;
        }
        */

        private string getEyeGazeData()
        {
            string objectHitName = "Background";
            Vector3 objectHitPos = Vector3.zero;
            var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;
            if (eyeGazeProvider != null)
            {
                if (eyeGazeProvider.IsEyeTrackingEnabledAndValid)
                {
                    if (eyeGazeProvider.HitPosition != null)
                    {
                        if (eyeGazeProvider.HitPosition.IsValidVector())
                        {
                            if(base_link != null)
                            {
                                objectHitPos = base_link.transform.InverseTransformPoint(eyeGazeProvider.HitPosition);
                            }
                            else
                            {
                                objectHitPos = eyeGazeProvider.HitPosition;
                            }
                        }
                    }
                    if(eyeGazeProvider.HitInfo.raycastValid)
                    {
                        if(eyeGazeProvider.HitInfo.transform != null)
                        {
                            if(eyeGazeProvider.HitInfo.transform.gameObject != null)
                            {
                                if (eyeGazeProvider.HitInfo.transform.gameObject.name != null)
                                {
                                        objectHitName = eyeGazeProvider.HitInfo.transform.gameObject.name;
                                }
                            }
                        }
                                
                    }
                }
            }
            if (objectHitName == null)
            {
                objectHitName = "Background";
            }
            else if (objectHitName.Contains("Spatial"))
            {
                objectHitName = "SpatialMesh";
            }
            if (objectHitPos == null)
            {
                objectHitPos = Vector3.zero;
            }
            var objectHitPosString = objectHitPos.ToString("F7");
            if (objectHitPosString == null)
                objectHitPosString = "";
            return objectHitName + seperator + objectHitPosString;
        }

        
        private string getHoloLensPosition()
        {
            Vector3 relativePos = Vector3.zero;
            Vector3 relativeOrientation = Vector3.zero;
            if (base_link != null && mainCamera != null)
            {
                relativePos = base_link.transform.InverseTransformPoint(mainCamera.transform.position);
                var relativeQuat = Quaternion.Inverse(base_link.transform.rotation) * mainCamera.transform.rotation;
                relativeOrientation = relativeQuat.eulerAngles;
            }
            //Debug.Log("RelativePos: " + relativePos.ToString());
            return relativePos.ToString("F7") + ";" + relativeOrientation.ToString("F7");
        }
        

        private void logData()
        {
            if(mainCamera == null || base_link == null)
            {
                getTransforms();
            }
            else
            {
                var hololensPos = getHoloLensPosition();
                //var hololensPos = mainCamera.transform.position.ToString("F7");
                var eyeData = getEyeGazeData();
                if (hololensPos == null)
                    hololensPos = "";
                if (eyeData == null)
                    eyeData = "";
                AddRow(hololensPos + seperator + eyeData);
            }
        }

        public void doSceneStart()
        {
            experimentStatusPub.publishMessage("Got image_target");
            AddRow("FUNCTION;IMAGE_TARGET_FOUND");
        }

        public void doSceneEnd()
        {
            experimentStatusPub.publishMessage("Scene task ended");
            AddRow("FUNCTION;END_TASK");
            FlushData();
        }
    }
}