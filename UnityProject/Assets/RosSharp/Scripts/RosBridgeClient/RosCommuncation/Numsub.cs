using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Numsub : UnitySubscriber<MessageTypes.Rob8.Num>
    {
        public float messageData;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Rob8.Num message)
        {
            messageData = message.num;
            Debug.Log(messageData);
        }
    }
}