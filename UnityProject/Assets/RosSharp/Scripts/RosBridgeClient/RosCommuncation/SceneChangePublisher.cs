using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SceneSystem;


namespace RosSharp.RosBridgeClient
{
    public class SceneChangePublisher : UnityPublisher<MessageTypes.Std.String>
    {
        public int messageData;
        private MessageTypes.Std.String message;
        public int ticks_between_publish = 1000;
        private int current_time = 0;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
            Publish(message);
        }

        private void InitializeMessage()
        {
            Debug.Log("Scene names: ");

            var sceneSystem = MixedRealityToolkit.Instance.GetService<IMixedRealitySceneSystem>();
            var sceneNames = sceneSystem.ContentSceneNames;

            string datanames = "";
            for (int x = 0; x < sceneNames.Length; x++)
            {
                string the_name = sceneNames[x];
                Debug.Log(the_name);
                if (the_name is null)
                {
                    continue;
                }
                if (datanames.Length > 2)
                {
                    datanames += ", ";
                }

                datanames += the_name + ":" + x.ToString();
            }
            
            message = new MessageTypes.Std.String
            {
                data = datanames
            };
        }

        private void Update()
        {
            current_time += 1;
            if (current_time > ticks_between_publish)
            {
                current_time = 0;
                Publish(message);
            }
        }
    }
}
