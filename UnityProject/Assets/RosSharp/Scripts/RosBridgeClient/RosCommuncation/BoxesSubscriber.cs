using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;
namespace RosSharp.RosBridgeClient
{
    public class BoxesSubscriber : UnitySubscriber<MessageTypes.Rob8.Boxes>
    {
        public long boxId;
        public Pose boxPose;
        public String boxType;


        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Rob8.Boxes message)
        {
            boxId = message.boxid;
            boxPose = message.pose;
            boxType = message.type;
           
        }
    }
}
