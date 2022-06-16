using System.Collections;
using System.Collections.Generic;
using RosSharp;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEngine;
using UnityEngine.Events;
namespace RosSharp.RosBridgeClient
{
    public class BoxesPublisher : UnityPublisher<MessageTypes.Rob8.Boxes>
    {
        
        public MessageTypes.Geometry.Pose pose_;
        private String type_;
        
       
        private UnityEngine.Vector3 scale_;
        private MessageTypes.Rob8.Boxes message;

        //public UnityEvent publishEvent;
       

        protected override void Start()
        {
            base.Start();

            /*if(publishEvent == null)
            {
                publishEvent = new UnityEvent();
            }
            publishEvent.AddListener(SendMessage);*/

            //type_ = new String(getType); 
            //scale_ = box.transform.localScale;

            //pose_ = new MessageTypes.Geometry.Pose();
            InitializeMessage();
            //SendMessage();

        }

        private void InitializeMessage()
        {
            pose_ = new MessageTypes.Geometry.Pose();
            message = new MessageTypes.Rob8.Boxes
            {
                boxid = 0,
                pose = pose_,
                type = type_,
                scalex = scale_[0],
                scaley = scale_[1],
                scalez = scale_[2]
            };

        }

        private void UpdateMessage(GameObject box, int id)
        {
            //https://github.com/siemens/ros-sharp/wiki/
            UnityEngine.Vector3 temp_vector = RosSharp.TransformExtensions.Unity2Ros(box.transform.localPosition);
            pose_.position.x = temp_vector.x;
            pose_.position.y = temp_vector.y;
            pose_.position.z = temp_vector.z;

            UnityEngine.Quaternion temp_quad = RosSharp.TransformExtensions.Unity2Ros(box.transform.localRotation);
            pose_.orientation.x = temp_quad.x;
            pose_.orientation.y = temp_quad.y;
            pose_.orientation.z = temp_quad.z;
            pose_.orientation.w = temp_quad.w;

            scale_ = box.transform.localScale;
            UnityEngine.Vector3 temp_scale = RosSharp.TransformExtensions.Ros2UnityScale(scale_);
            message.scalex = temp_scale[0];
            message.scaley = temp_scale[1];
            message.scalez = temp_scale[2];
            message.boxid = id;
            type_ = new String(box.GetComponent<boxparam>().type);
            message.type = type_;
            message.pose = pose_;

        }

        public void SendMessage(GameObject box)
        {
            foreach(var id in box.GetComponent<boxparam>().ids)
            {
                UpdateMessage(box, id);
                Publish(message);
            }
            //Debug.Log("send msg");
        }
    }
}
