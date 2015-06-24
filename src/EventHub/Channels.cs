using Amqp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Argonaut.Networking
{
    public class ChannelBase
    {
        public Connection Connection { get; set; }
        public Session Session { get; set; }
        public string PartitionId { get; set; }
        public string Name { get; set; }
    }
    public class SendChannel : ChannelBase
    {
        public SenderLink Link { get; set; }
    }
    public class ReceiveChannel : ChannelBase
    {
        public ReceiverLink Link { get; set; }
    }
}
