from google.cloud import pubsub_v1
import time
import datetime

project_id = "GCP-project-name"
topic_name = "GCP-topic-name"

publisher = pubsub_v1.PublisherClient()
topic_path = publisher.topic_path(project_id, topic_name)
print('topic_path: {}'.format(topic_path))

for n in range(1, 10000):
    data = u'{"source":"SAMPLE_MSG_FOR_TESTING","values":[{"wellhead_pressure_1":9.314,"slurry_rate":0.0,"pumpside_pressure":5.45,"inline_density_1":0.0,"inline_density_2":0.0,"blender_density_1":0.0,"clean_rate_1":0.0,"total_clean_volume":0.0,"total_proppant":0.0,"stage_total_proppant":0.0,"stage_slurry_volume":0.0,"timestamp":1.5465527742741542E9,"timestamp_local":1.546527574274154E9,"timestamp_offset":-21600.0,"system_status":1,"well_name":"Basilisk/Cerberus 56-3-43 Unit 1H","api":"42 - 389 - 36045","stage":24}]}'
    # Data must be a bytestring
    data = data.encode('utf-8')
    # Add two attributes, frac_device_name and publishTime, to the message
    publisher.publish(topic_path, data=data, frac_device_name='frac-test-001', publishTime=datetime.datetime.utcnow().isoformat())
    print('Published messages with custom attributes. {}'.format(data))
    time.sleep(1)
