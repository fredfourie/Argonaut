using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Argonaut.Networking
{
    using Amqp;

    public class EventHubSettings
    {
        const string AzureServiceBusNamespaceSuffix = "servicebus.windows.net";
        const int DefaultAmqpPort = 5671;

        public string ServiceBusName { get; set; }
        public string Namespace
        {
            get
            {
                return string.Format("{0}.{1}", ServiceBusName, AzureServiceBusNamespaceSuffix);
            }
        }

        public string EventHubName { get; set; }
        public Dictionary<string, string> Policies { get; set; }
        public int AmqpPort { get; set; }

        public Address GetPolicyAddress(string policyName)
        {
            string policyKey = string.Empty;
            if (!Policies.TryGetValue(policyName, out policyKey))
            {
                throw new ArgumentException("Policy name not found.");
            }

            return new Address(Namespace, AmqpPort, policyName, policyKey);
        }

        public EventHubSettings()
        {
            AmqpPort = DefaultAmqpPort;
        }
    }
}
