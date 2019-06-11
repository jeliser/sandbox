# pip3 install pick
from pick import pick
import subprocess
options = {'Microsoft Azure': 'azure azure-eventhub', 'Google Cloud Platform (GCP)' : 'google-cloud-storage' }
for selected in pick(list(options.keys()), 'Select optional packages to install (press SPACE to mark, ENTER to continue):', multi_select=True, min_selection_count=0):
    subprocess.call('python3 -m pip install {}'.format(options[selected[0]]), shell=True)
