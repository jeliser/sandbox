

import subprocess
p = subprocess.Popen("yum list installed".split(" "), stdout=subprocess.PIPE)
contents = p.communicate()[0].split("\n")

package_list = ""

start = False
for row in contents:
  if len(row) <= 0:
    pass
  elif start and row[0] != " ":
    package_list += row.split(".")[0] + " "
  elif(row.find("Installed Packages") >= 0):
    start = True

print package_list

