#Documentation about SCons build scripts 
#https://docs.platformio.org/en/latest/projectconf/advanced_scripting.html
#https://scons.org/doc/production/HTML/scons-user.html

Import("env")
import datetime

#Change the name of the output firmware file to include relevant information
date = datetime.datetime.now().strftime("%Y-%m-%d")
env.Replace(PROGNAME=f"{env.get('PIOENV')}-{date}")