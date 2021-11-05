#!/usr/bin/env python

import os
import re
import argparse
import traceback
import json
import random

import pyutils.utils as utils

from box import Box  ## pip install python-box

EXAMPLES = [
  '{"BlockDiagramName":"sldemo_antiwindupfeedforward","FileName":"/home/jeliser/Documents/MATLAB/Examples/R2021a/simulink_industrial/AntiWindupControlUsingAPIDControllerExample/sldemo_antiwindupfeedforward.slx","BlockDiagramType":"Model","IsLibrary":false,"SimulinkVersion":"10.3","ReleaseName":"R2021a","ReleaseUpdateLevel":0,"ModelVersion":"2.0","LastModifiedBy":"The MathWorks, Inc.","Description":"","SavedCharacterEncoding":"","LastSavedArchitecture":"glnxa64","Metadata":[],"Interface":{"Inports":[],"Outports":{"Name":"yout","SignalName":"y(t)","DisplayName":"yout","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},"Trigports":[],"Enableports":[],"ModelVersion":"2.0","SubsystemReferences":[],"ModelReferences":[],"ParameterArgumentNames":"","TestPointedSignals":[],"ProvidedFunctions":[],"IsExportFunctionModel":false,"SimulinkSubDomainType":"Simulink","ResetEvents":[],"HasInitializeEvent":false,"HasTerminateEvent":false,"PreCompExecutionDomainType":"Unset","ParameterArguments":[],"ExternalFileReference":{"Reference":"pid_lib/PID Controller","Path":"sldemo_antiwindupfeedforward/PID Controller","Type":"Library_Link","SID":"3"}}}',
  '{"BlockDiagramName":"sldemo_hardstop","FileName":"/home/jeliser/Documents/MATLAB/Examples/R2021a/simulink_general/sldemo_hardstopExample/sldemo_hardstop.slx","BlockDiagramType":"Model","IsLibrary":false,"SimulinkVersion":"10.3","ReleaseName":"R2021a","ReleaseUpdateLevel":0,"ModelVersion":"5.0","LastModifiedBy":"The MathWorks, Inc.","Description":"Friction Model with Hard Stops\n\nThis example shows how to model friction one way in Simulink(R). The two integrators in \nthe model calculate the velocity and position of the system, which is then used in \nthe Friction Model to calculate the friction force. \n\nRunning the simulation shows the initial condition response on the Scope. ","SavedCharacterEncoding":"","LastSavedArchitecture":"glnxa64","Metadata":[],"Interface":{"Inports":[],"Outports":[{"Name":"X","SignalName":"X","DisplayName":"X","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"Against Limit","SignalName":"","DisplayName":"Against Limit","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"F_fr","SignalName":"","DisplayName":"F_fr","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"Stuck","SignalName":"","DisplayName":"Stuck","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"}],"Trigports":[],"Enableports":[],"ModelVersion":"5.0","SubsystemReferences":[],"ModelReferences":[],"ParameterArgumentNames":"","TestPointedSignals":[],"ProvidedFunctions":[],"IsExportFunctionModel":false,"SimulinkSubDomainType":"Simulink","ResetEvents":[],"HasInitializeEvent":false,"HasTerminateEvent":false,"PreCompExecutionDomainType":"Unset","ParameterArguments":[],"ExternalFileReference":[]}}',
]

def main():
  parser = argparse.ArgumentParser(description='Test application for the run command library')
  parser.add_argument('-v', '--verbose', help='Verbose output', action='store_true', default=False)
  parser.add_argument(      '--dryrun', help='Dryrun the commands', action='store_true', default=False)

  parser.add_argument('-m', '--model', help = 'The Simulink Model to extract the information from', default=None)
  parser.add_argument('-e', '--example', help = 'Which preloaded Simulink Model example to use. (0 - {})'.format(len(EXAMPLES)-1), default=None)

  args = parser.parse_args() 

  if args.model:
    if os.path.isfile(args.model):
      cmd = "matlab -nosplash -nodesktop -r \"try; disp('ans ='); disp(jsonencode(Simulink.MDLInfo('{}'))); catch; end; exit\";".format(args.model)
      raw = utils.run_cmd(cmd)
      raw = str(re.search('{(.*)}', utils.run_cmd(cmd)).group(0))
    else:
      print('"{}" does not exist'.format(args.model))
      return
  else:
    raw = EXAMPLES[random.randint(0, len(EXAMPLES)-1) if not args.example else int(args.example)]

  # Sanitize the inputs
  raw = raw.replace('\n', '\\n')
  model = Box(json.loads(raw))

  # Convert the metadata of the loaded model into something that can be parsed by downstream consumers.
  metadata = Box()
  for key, port_type in [['inputs', model.Interface.Inports], ['outputs', model.Interface.Outports]]:
    # Because JSON parsing is garbage :( -- https://www.mathworks.com/matlabcentral/answers/423553-json-decode-encode-eats-arrays
    # Need to promote the type to a list if it's not already.
    if not isinstance(port_type, list):
      port_type = [port_type]

    # Create the keys and pull the fields over into the new structure.
    metadata[key] = []
    for port in port_type:
      metadata[key].append({'name': port.Name, 'description': port.DisplayName, 'type': port.DataTypeExpr if port.DataTypeExpr else 'float64'})
  print(metadata)

  # Convert to YAML
  #print yaml.dump(yaml.load(json.dumps(json.loads(open(sys.argv[1]).read()))), default_flow_style=False)


if __name__ == "__main__":
  try:
    main()
  except:
    traceback.print_exc()

