import collections
import yaml

d = dict()
d['name'] = 'Record1'
d['description'] = 'This is a description about the message'
d['fields'] = [
        {'name': 'field_1', 'symbol': 'ABC', 'min': 100, 'description': 'This describes field 1'},
        {'name': 'field_2', 'symbol': 'DEF', 'max': 300, 'description': 'This describes field 2'},
        {'name': 'field_3', 'symbol': 'GHI', 'description': 'This describes field 3'}
        ]

print yaml.dump(d)
print yaml.load(yaml.dump(d))
