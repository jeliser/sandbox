import os
from google.cloud import pubsub_v1

project_id = 'external-kelvin'
topic_name = 'connection-stats-kelvin-sub-001'

subscriber = pubsub_v1.SubscriberClient()
topic_name = 'projects/{}/subscriptions/{}'.format(project_id, topic_name)

def callback(message):
    print(message.data)
    message.ack()

# As before, substitute {project} and {subscription} with appropriate
# values for your application.
future = subscriber.subscribe(topic_name, callback)

try:
    future.result()
except KeyboardInterrupt:
    future.cancel()
