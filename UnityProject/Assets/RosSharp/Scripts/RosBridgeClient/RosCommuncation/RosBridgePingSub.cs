using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using holoutils;

namespace RosSharp.RosBridgeClient
{
    public class RosBridgePingSub : UnitySubscriber<MessageTypes.Std.Bool>
    {
        bool data;
        float msgTime;
        public bool lostConnection;
        public CSVLogger logger;

        protected override void Start()
        {
            base.Start();
            //msgTime = Time.time;
        }

        protected override void ReceiveMessage(MessageTypes.Std.Bool message)
        {
            //msgTime = Time.time;
            if (logger != null)
            {
                logger.doEnd = true;
            }
        }
    }
}
