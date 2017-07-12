# Robot Waitress UI

The UI is initialised using the file `waitress_gui.py` as the entry point. The GUI can be accessed by any suitable device on the same LAN as LUCIE. This enables admin staff to issue LUCIE with commands and monitor her behaviour from a stationary machine.

`rosrun waitress_ui waitress_gui.py`

## Configuration Options
There are constants that can be edited at the top of `waitress_gui.py`:
* `HUB = 'WayPoint1'` tells LUCIE which WayPoint to use as the hub, i.e. where to collect delivery items from.
* `NUMBER_OF_WAYPOINTS = 14` tells the UI how many WayPoints are in the Map.
* `ONE_MACHINE = True` Run the UI solely on LUCIE, this will automatically login for admin features.
* `PIN = 1111` set this to a new code, it is what admin staff can use to login. This is not a secure method.
* `WONDERING_MODE = True` make LUCIE go to a random node inbetween orders. If False, LUCIE will return to the HUB.
* `TWITTER = True` provides the link to the Twitter page, use this if the Twitter program is intended to be used.

## Menu Options
The menu is read in from `menu.json`. Each item has a name and an image file name, these images need to be located in the `static/images` directory.
