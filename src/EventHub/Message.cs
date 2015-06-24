using System;
namespace Argonaut.Networking
{
    public class Message
    {
        public string Offset { get; private set; }
        public DateTime EnqueuedTimeUtc { get; private set; }
        public long SequenceNumber { get; private set; }
        public object Body { get; private set; }

        public Message(string offset, DateTime enqueuedTimeUtc, long sequenceNumber, object body)
        {
            this.Offset = offset;
            this.EnqueuedTimeUtc = enqueuedTimeUtc;
            this.SequenceNumber = sequenceNumber;
            this.Body = body;
        }
    }
}
