using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;
using System;

namespace RosSharp.RosBridgeClient
{
    public class SceneChangesub : UnitySubscriber<MessageTypes.Std.Int16>
    {
        public int messageData = -1;

        protected override void Start()
        {
            base.Start();
// messageData = -1;
        }

        protected override void ReceiveMessage(MessageTypes.Std.Int16 message)
        {

            messageData = message.data;
            Debug.Log(messageData);
        }
    }
}
