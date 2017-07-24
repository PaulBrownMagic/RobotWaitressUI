# Robot Waitress UI

The UI is initialised using the file `waitress_gui.py` as the entry point. The GUI can be accessed by any suitable device on the same LAN as LUCIE. This enables admin staff to issue LUCIE with commands and monitor her behaviour from a stationary machine.

`rosrun waitress_ui waitress_gui.py`

## Configuration Options
There are constants that can be edited in `config.py`:
* `MENU = [{"name": "Foo", "image": "foo.jpg"}]` used to create the menu.
Each item has a name and an image file name. All image files should be available in the `static/images/` directory.
* `HUB = 'WayPoint1'` tells LUCIE which WayPoint to use as the hub, i.e. where to collect delivery items from.
* `NUMBER_OF_WAYPOINTS = 14` tells the UI how many WayPoints are in the Map.
* `ONE_MACHINE = True` Run the UI solely on LUCIE, this will automatically login for admin features.
* `PIN = 1111` set this to a new code, it is what admin staff can use to login. This is not a secure method.
* `WONDERING_MODE = True` make LUCIE go to a random node inbetween orders. If False, LUCIE will return to the HUB.
* `TWITTER = True` provides the link to the Twitter page, use this if the Twitter program is intended to be used.
* `STRANDS_UI_URL` allows the existing helper functions to be embedded in the generic navigation page. This is the url given when strands_ui is launched.
