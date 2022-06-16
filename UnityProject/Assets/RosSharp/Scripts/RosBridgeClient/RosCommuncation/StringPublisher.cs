using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SceneSystem;


namespace RosSharp.RosBridgeClient
{
    public class StringPublisher : UnityPublisher<MessageTypes.Std.String>
    {
        private MessageTypes.Std.String message;
        public int ticks_between_publish = 1000;
        private int current_time = 0;


        protected override void Start()
        {
            base.Start();
            InitializeMessage("");
        }

        private void InitializeMessage(string _data)
        {            
            message = new MessageTypes.Std.String
            {
                data = _data
            };
        }

        public void publishMessage(string _data)
        {
            // Debug.Log("Trying to publish /experiment/status msg");
            InitializeMessage(_data);
            Publish(message);
        }
    }
}
