using System;
using System.Collections.Generic;

namespace Argonaut.Networking
{
    using Amqp;
    using Amqp.Framing;
    using Amqp.Types;


    public class EventHubClient
    {
        // EventHub attributes
        protected EventHubSettings settings = null;
        protected string policyName = null;
        protected List<string> partitions = new List<string>();

        // AMQP attributes
        protected Connection amqpConnection = null;
        protected Session amqpSession = null;

        public EventHubClient(EventHubSettings settings, string sendPolicyName)
        {
            this.settings = settings;
            this.policyName = sendPolicyName;
        }

        protected void InitializeConnectionAndSession()
        {
            Address address = settings.GetPolicyAddress(this.policyName);
            this.amqpConnection = new Connection(address);
            this.amqpSession = new Session(this.amqpConnection);

            // Get the event hub partitions. This will also serve as a canary
            // in the event of connectivity problems
            GetPartitions(this.amqpSession);
            if (this.partitions.Count == 0)
            {
                throw new Exception("No valid event hub partitions found.");
            }
        }

        protected void GetPartitions(Session session)
        {
            ReceiverLink receiverLink = null;
            SenderLink senderLink = null;

            try
            {
                // create a pair of links for request/response
                Trace.WriteLine(TraceLevel.Information, "Creating a request and a response link...");
                string clientNode = "client-temp-node";
                senderLink = new SenderLink(session, "mgmt-sender", "$management");
                receiverLink = new ReceiverLink(
                    session,
                    "mgmt-receiver",
                    new Attach()
                    {
                        Source = new Source() { Address = "$management" },
                        Target = new Target() { Address = clientNode }
                    },
                    null);

                var request = new Amqp.Message();
                request.Properties = new Properties() { MessageId = "request1", ReplyTo = clientNode };
                request.ApplicationProperties = new ApplicationProperties();
                request.ApplicationProperties["operation"] = "READ";
                request.ApplicationProperties["name"] = settings.EventHubName;
                request.ApplicationProperties["type"] = "com.microsoft:eventhub";
                senderLink.Send(request, null, null); 

                var response = receiverLink.Receive(5000); // time out after 5 seconds
                if (response == null)
                {
                    throw new Exception("No get partitions response was received.");
                }

                receiverLink.Accept(response);

                Trace.WriteLine(TraceLevel.Information, "Partition info {0}", response.Body.ToString());
                var partitionStrings = (string[])((Map)response.Body)["partition_ids"];
                Trace.WriteLine(TraceLevel.Information, "Partitions {0}", string.Join(",", partitionStrings));
                this.partitions = new List<string>(partitionStrings);
            }
            catch (Exception x)
            {
                Trace.WriteLine(TraceLevel.Error, "Error retrieving partitions:\r\n{0}", x.ToString());
            }
            finally
            {
                if (receiverLink != null) receiverLink.Close();
                if (senderLink != null) senderLink.Close();
            }
        }

    }
}
