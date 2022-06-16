

using UnityEngine;
using System;
using System.Collections;



namespace RosSharp.RosBridgeClient
{
    public class target_box_behavior : UnitySubscriber<MessageTypes.Geometry.PoseStamped>
    {
        public Transform PublishedTransform;
        public GameObject target_box;
        private Vector3 position;
        private Quaternion rotation;
        private bool isMessageReceived;
        private Renderer cubeRenderer;
        private float timeOfMessage;


        protected override void Start()
        {
            base.Start();
            cubeRenderer = target_box.GetComponent<Renderer>();
            cubeRenderer.enabled = (false);
            StartCoroutine(Fade());
            //timeOfMessage = Time.time;

        }

        private void Update()
        {
            if (isMessageReceived)
            {
                Debug.Log("msg!");
                ProcessMessage();
                cubeRenderer.enabled = (true);
                timeOfMessage = Time.time;
                isMessageReceived = false;
            }

            if (Time.time - timeOfMessage > + 5.0)
            {
                cubeRenderer.enabled = (false);

            }
        }

        IEnumerator Fade()
        {
            while (true) 
            {
                Color c = cubeRenderer.material.color;
                for (float alpha = 1f; alpha >= 0; alpha -= 0.1f)
                {
                    c.a = alpha;
                    cubeRenderer.material.color = c;
                    yield return new WaitForSeconds(.1f);
                }
                for (float alpha = 0f; alpha <= 1; alpha += 0.1f)
                {
                    c.a = alpha;
                    cubeRenderer.material.color = c;
                    yield return new WaitForSeconds(.1f);
                }
            }
            
        }

        

        protected override void ReceiveMessage(MessageTypes.Geometry.PoseStamped message)
        {
            position = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            PublishedTransform.position = position;
            PublishedTransform.rotation = rotation;
        }

        private Vector3 GetPosition(MessageTypes.Geometry.PoseStamped message)
        {
            return new Vector3(
                (float)message.pose.position.x,
                (float)message.pose.position.y,
                (float)message.pose.position.z);
        }

        private Quaternion GetRotation(MessageTypes.Geometry.PoseStamped message)
        {
            return new Quaternion(
                (float)message.pose.orientation.x,
                (float)message.pose.orientation.y,
                (float)message.pose.orientation.z,
                (float)message.pose.orientation.w);
        }
    }
}