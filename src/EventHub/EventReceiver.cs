using System;
using System.Collections.Generic;

namespace Argonaut.Networking
{
    using Amqp;
    using Amqp.Framing;
    using Amqp.Types;

    public class EventReceiver : EventHubClient
    {
        public string Name { get; set; }
        public int ReceiveTimeoutMs { get; set; }
        
        protected Dictionary<string, ReceiveChannel> channels =
            new Dictionary<string, ReceiveChannel>();
        
        public EventReceiver(string name, EventHubSettings settings, string receivePolicyName, int receiveTimeoutMs = 5000) :
            base(settings, receivePolicyName)
        {
            this.Name = name;
            this.ReceiveTimeoutMs = receiveTimeoutMs;
        }

        public void InitializeChannels(ReceiveLinkDescription[] receiveLinks)
        {
            InitializeConnectionAndSession();

            foreach (var link in receiveLinks)
            {
                // Some sanity checks first
                if (link.Type != LinkType.Receive) continue;
                if (!this.partitions.Contains(link.PartitionId.ToString()))
                {
                    throw new Exception(string.Format(
                        "Partition '{0}' does not exist in this event hub and cannot be initialized.",
                        link.PartitionId));
                }

                string partitionAddress = string.Format(
                    "{0}/ConsumerGroups/{1}/Partitions/{2}", 
                    this.settings.EventHubName,
                    link.ConsumerGroup,
                    link.PartitionId);

                var filters = BuildFilter(link);
                var amqpLink = new ReceiverLink(
                   this.amqpSession,
                   string.Format("receiver-{0}-{1}", this.Name, link.PartitionId),
                   new Source()
                   {
                       Address = partitionAddress,
                       FilterSet = filters
                   },
                   null);

                this.channels.Add(link.Name, new ReceiveChannel()
                {
                    Connection = amqpConnection,
                    Session = amqpSession,
                    Link = amqpLink,
                    Name = link.Name,
                    PartitionId = link.PartitionId.ToString()
                });
            }
        }

        protected Map BuildFilter(ReceiveLinkDescription receiveLink)
        {
            string filter = string.Empty;
            Map filters = null;

            if (receiveLink.SequenceCheckPoint > -1)
            {
                filter = string.Format(
                "amqp.annotation.x-opt-offset > {0}",
                receiveLink.SequenceCheckPoint);
            }
            else if (receiveLink.TimeCheckPoint > DateTime.MinValue)
            {
                // Get milliseconds since start of 1970
                long msOffset = (long)receiveLink.TimeCheckPoint.Subtract(
                    new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds;
                filter = string.Format(
                    "amqp.annotation.x-opt-enqueuedtimeutc > {0}",
                    msOffset);
            }

            if (!string.IsNullOrWhiteSpace(filter))
            {
                filters = new Map();
                    filters.Add(new Symbol("apache.org:selector-filter:string"),
                        new DescribedValue(
                            new Symbol("apache.org:selector-filter:string"),
                            filter));
            }

            return filters;
        }

        /// <summary>
        /// Synchronous read function.
        /// </summary>
        /// <param name="channelName">The name of the channel to read from.</param>
        /// <returns>An instance of the <see cref="Argonaut.Networking.Message"/> class.</returns>
        public Message ReadMessage(string channelName)
        {
            var channel = this.channels[channelName];

            Amqp.Message amqpMessage = null;
            try
            {
                amqpMessage = channel.Link.Receive(this.ReceiveTimeoutMs);
            }
            catch (Exception x)
            {
                //Console.WriteLine(x.Message);
            }

            if (amqpMessage == null)
            {
                return null;
            }

            channel.Link.Accept(amqpMessage);
            string offset = (string)amqpMessage.MessageAnnotations[new Symbol("x-opt-offset")];
            long seqNumber = (long)amqpMessage.MessageAnnotations[new Symbol("x-opt-sequence-number")];
            DateTime enqueuedTime = (DateTime)amqpMessage.MessageAnnotations[new Symbol("x-opt-enqueued-time")];
            var message = new Message(offset, enqueuedTime, seqNumber, amqpMessage.Body);
            return message;
        }


    }
}
