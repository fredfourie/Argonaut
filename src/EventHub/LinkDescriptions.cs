
using System;
namespace Argonaut.Networking
{
    public class LinkDescription
    {
        public string Name { get; set; }
        public LinkType Type { get; set; }
        public string PartitionId { get; set; }

        public LinkDescription(string name, LinkType type, string partitionId)
        {
            this.Name = name;
            this.Type = type;
            this.PartitionId = partitionId;
        }
    }

    public class SendLinkDescription : LinkDescription
    {
        public SendLinkDescription(
            string name,
            string partitionId = "0") :
            base(name, LinkType.Send, partitionId)
        { }
    }

    public class ReceiveLinkDescription : LinkDescription
    {
        public string ConsumerGroup { get; set; }
        /// <summary>
        /// UTC Enqueue time checkpoint
        /// </summary>
        public DateTime TimeCheckPoint { get; set; }
        /// <summary>
        /// Numerical sequence identifier checkpoint
        /// </summary>
        public long SequenceCheckPoint { get; set; }

        public ReceiveLinkDescription(
            string name, 
            string partitionId = "0",
            string consumerGroup = "$default") :
            base(name, LinkType.Receive, partitionId)
        {
            this.ConsumerGroup = consumerGroup;
            this.SequenceCheckPoint = -1;
            this.TimeCheckPoint = DateTime.MinValue;
        }
    }
}
