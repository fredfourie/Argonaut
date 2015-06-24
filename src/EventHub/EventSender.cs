using System;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Text;

namespace Argonaut.Networking
{
    using Amqp;
    using Amqp.Framing;
    using Amqp.Types;

    public class EventSender : EventHubClient
    {
        public string Name { get; set; }
        protected Dictionary<string, SendChannel> channels =
            new Dictionary<string, SendChannel>();

        public EventSender(string name, EventHubSettings settings, string sendPolicyName)
            : base(settings, sendPolicyName)
        {
            this.Name = name;
        }

        public void InitializeChannels(SendLinkDescription[] sendLinks)
        {
            InitializeConnectionAndSession();

            foreach(var link in sendLinks)
            {
                // Some sanity checks first
                if (link.Type != LinkType.Send) continue;
                if (!this.partitions.Contains(link.PartitionId.ToString()))
                {
                    throw new Exception(string.Format(
                        "Partition '{0}' does not exist in this event hub and cannot be initialized.", 
                        link.PartitionId));
                }

                var amqpLink = new SenderLink(
                    this.amqpSession,
                    "send-link:" + link.Name,
                    string.Format("{0}/Partitions/{1}", 
                        this.settings.EventHubName, 
                        link.PartitionId));

                this.channels.Add(link.Name, new SendChannel() 
                { 
                    Connection = amqpConnection,
                    Session = amqpSession,
                    Link = amqpLink, 
                    Name = link.Name,
                    PartitionId = link.PartitionId.ToString()
                });
            }
        }

        public void SendEvent(string linkName, string content)
        {
            this.SendEvent(linkName, Encoding.UTF8.GetBytes(content));
        }

        public void SendEvent(string linkName, byte[] content)
        {
            var channel = this.channels[linkName];

            var messageProperties = new Properties()
            {
                GroupId = string.Format("send-{0}-p{1}", channel.Name.ToLower(), channel.PartitionId)
            };

            var message = new Amqp.Message()
            {
                BodySection = new Data() { Binary = content },
                Properties = messageProperties,
                MessageAnnotations = new MessageAnnotations()
            };

            message.MessageAnnotations[new Symbol("x-opt-partition-key")] = "pk:" + channel.PartitionId;

            channel.Link.Send(message);
        }
    }
}
