# Robot Waitress UI

The UI is initialised using the file `waitress_gui.py` as the entry point. The GUI can be accessed by any suitable device on the same LAN as LUCIE. This enables admin staff to issue LUCIE with commands and monitor her behaviour from a stationary machine.

## Configuration Options
There are two constants that can be edited at the top of `waitress_gui.py`:
* `TWITTER = True` provides the link to the Twitter page, use this if the Twitter program is intended to be used.
* `PIN = 1111` set this to a new code, it is what admin staff can use to login. This is not a secure method.

## Menu Options
The menu is read in from `menu.json`. Each item has a name and an image file name, these images need to be located in the `static/images` directory. 
