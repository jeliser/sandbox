#!/usr/bin/env python3
from pprint import pprint

import pypd
from pypd.errors import BadRequest

pypd.api_key = "Nszy9xESLmK3VZ9sqoqE"

# REF  -> https://v2.developer.pagerduty.com/docs/send-an-event-events-api-v2
# POST -> https://events.pagerduty.com/v2/enqueue
data={
    'routing_key': 'R0138KARE8Z7AQ7WEIFYY7YF70RHC04M',
    'event_action': 'trigger',
    'payload': {
        'summary': 'this is an error event!',
        'severity': 'critical',
        'source': '5d81a54f-9e28-409b-9347-d9103b554a0c',
        'component': 'serial',
        'group': 'WITS0-driver',
        'class': 'disconnection',
    }
}

'''
curl -X POST --header 'Content-Type: application/json' --header 'Accept: application vnd.pagerduty+json;version=2' --header 'Authorization: Token token=Nszy9xESLmK3VZ9sqoqE' -d '{
    "routing_key": "R0138KARE8Z7AQ7WEIFYY7YF70RHC04M",
    "event_action": "trigger",
    "payload": {
        "summary": "Serial disconnection",
        "severity": "critical",
        "source": "5d81a54f-9e28-409b-9347-d9103b554a0c",
        "component": "serial",
        "group": "WITS0-driver",
        "class": "disconnection",
        "custom_details": {
            "device": "/dev/ttyS01",
            "baud": "9600"
        }
    },
    "images": [{
  	"src": "https://www.pagerduty.com/wp-content/uploads/2016/05/pagerduty-logo-green.png",
  	"href": "https://example.com/",
  	"alt": "Example text"
    }],
    "links": [{
      "href": "https://example.com/",
      "text": "Link text"
    }]
}
' 'https://events.pagerduty.com/v2/enqueue'
'''

#print(data)
#pypd.EventV2.create(data=data)

# https://support.pagerduty.com/docs/pd-cef#section-pd-cef-fields
data = {
    'type': 'incident',
    'title': 'Sensor disconnect',
    'incident_key': 'cecd7ad4-3f8f-11e9-b210-d663bd873d93',
    'body': {
        'type': 'incident_body',
        'details': '/dev/ttyS0 disconnected and is reconnecting',
    },
}

'''
curl -X POST --header 'Content-Type: application/json' --header 'Accept: application vnd.pagerduty+json;version=2' --header 'From: joshua.eliser@kelvininc.com' --header 'Authorization: Token token=Nszy9xESLmK3VZ9sqoqE' -d '{
  "incident": {
    "type": "incident",
    "title": "Sample cURL",
    "service": {
      "id": "PGMHGN9",
      "type": "service_reference"
    }
  }
}
' 'https://api.pagerduty.com/incidents'
'''

# if the incident is already open it will error with BadRequest
try:
    incident = pypd.Incident.create(
        data=data,
    )
except BadRequest:
    pass
    #incident = pypd.Incident.find(incident_key='cecd7ad4-3f8f-11e9-b210-d663bd873d93')[-1]

incidents = pypd.Incident.find(maximum=10)
print(incidents)


