import datetime
import os
from os.path import expanduser


KID_IDENTITY = "thibault_asselborn_25"
home = expanduser("~")
date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

os.makedirs(home + "/outputMemory/" + KID_IDENTITY)

# save action logs
with open(home + "/outputMemory/outputActions_" + "1252" + ".txt", "w") as f:
	f.write("winner is the player: ")
	f.close()