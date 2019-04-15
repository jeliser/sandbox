#!/usr/bin/env python3
import re

txt = '''
hello I would world
hello like to see world
hello this as a string world
'''

regex = '(?<=hello)(.*)(?=world)'

x = re.findall(regex, txt) 
print(x)


txt = '''
      // Check if we found a row delimiter
      if(properties.delim_row.empty()) {
        LOG_ERROR("Failed to find a matching row delimiter.\n%s\n", buff.c_str());
        properties.delim_field = "";
        publish_event(LoggingDefault::Level::WARN, EventSubsystem::ALGORITHM,
EventStatus::MISMATCH, "parser");
        return;
      }
    }

    // Process the rows in the buffer
    const auto& rows = Utils::split(buff, properties.delim_row);
    for(const auto& row : rows) {
      const auto& vals = Utils::split(row, properties.delim_field);
      if(vals.size() == 1 && strlen(vals[0].c_str()) == 0) {
        // This is a completely empty row, just continue
        end += row.size() + 1;
        continue;
      } else if(vals.size() != properties.fields.size()) {
        // Reset the buffer delimiters and let it attempt to choose new delimiters
        LOG_ERROR("Expected number of elements does not match: %d != %d\n", vals.size(), properties.fields.size());
        LOG_ERROR("%s\n", row.c_str());
        properties.delim_row = "";
        properties.delim_field = "";
        publish_event(
LoggingDefault::Level::ERROR, 
			EventSubsystem::ALGORITHM,
    EventStatus::MISMATCH, "parser");
        break;
      }
'''

regex = '(?<=publish_event\()(.*)(?=\)\;)'

x = re.findall(regex, re.sub('\s+', ' ', txt).strip().replace(');', ');\n').replace(' ', ''))
print(x)

print('')
for event in x:
  print(event)

filename = '/home/jeliser/code/automau5/cpp/protocols/DelimiterParser/DelimiterParser.cpp'
print(filename)
with open(filename, 'r') as file:
  data = file.read().replace('\n', '')

  x = re.findall(regex, re.sub('\s+', ' ', data).strip().replace(');', ');\n').replace(' ', ''))
  for event in x:
    print(event)
