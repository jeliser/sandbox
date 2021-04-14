#!/usr/bin/env python

import os
import re
import argparse
import traceback
import json

import pyutils.utils as utils

def main():
  parser = argparse.ArgumentParser(description='Test application for the run command library')
  parser.add_argument('-v', '--verbose', help='Verbose output', action='store_true', default=False)
  parser.add_argument(      '--dryrun', help='Dryrun the commands', action='store_true', default=False)

  parser.add_argument('-m', '--model', help = 'The Simulink Model to extract the information from', default=None)

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
    raw = '{"BlockDiagramName":"sldemo_hardstop","FileName":"/home/jeliser/Documents/MATLAB/Examples/R2021a/simulink_general/sldemo_hardstopExample/sldemo_hardstop.slx","BlockDiagramType":"Model","IsLibrary":false,"SimulinkVersion":"10.3","ReleaseName":"R2021a","ReleaseUpdateLevel":0,"ModelVersion":"5.0","LastModifiedBy":"The MathWorks, Inc.","Description":"Friction Model with Hard Stops\n\nThis example shows how to model friction one way in Simulink(R). The two integrators in \nthe model calculate the velocity and position of the system, which is then used in \nthe Friction Model to calculate the friction force. \n\nRunning the simulation shows the initial condition response on the Scope. ","SavedCharacterEncoding":"","LastSavedArchitecture":"glnxa64","Metadata":[],"Interface":{"Inports":[],"Outports":[{"Name":"X","SignalName":"X","DisplayName":"X","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"Against Limit","SignalName":"","DisplayName":"Against Limit","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"F_fr","SignalName":"","DisplayName":"F_fr","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"},{"Name":"Stuck","SignalName":"","DisplayName":"Stuck","BusObject":"","BusOutputAsStruct":false,"OutportClientType":false,"UnitExpr":"inherit","DataTypeExpr":"","ComplexityExpr":"","DimensionsExpr":"[-1]"}],"Trigports":[],"Enableports":[],"ModelVersion":"5.0","SubsystemReferences":[],"ModelReferences":[],"ParameterArgumentNames":"","TestPointedSignals":[],"ProvidedFunctions":[],"IsExportFunctionModel":false,"SimulinkSubDomainType":"Simulink","ResetEvents":[],"HasInitializeEvent":false,"HasTerminateEvent":false,"PreCompExecutionDomainType":"Unset","ParameterArguments":[],"ExternalFileReference":[]}}';

  # Sanitize the inputs
  raw = raw.replace('\n', '\\n')
  print(json.loads(raw))

if __name__ == "__main__":
  try:
    main()
  except:
    traceback.print_exc()

