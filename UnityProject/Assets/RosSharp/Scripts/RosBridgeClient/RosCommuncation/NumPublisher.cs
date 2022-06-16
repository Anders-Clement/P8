using System.Collections;
using System.Collections.Generic;
namespace RosSharp.RosBridgeClient
{
    public class NumPublisher : UnityPublisher<MessageTypes.Rob8.Num>
    {
        public int messageData;

        private MessageTypes.Rob8.Num message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Rob8.Num
            {
                num = messageData
            };
        }

        private void Update()
        {
            message.num = messageData;
            Publish(message);
        }
    }
}
