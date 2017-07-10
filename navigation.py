import subprocess


class Navigation(object):
    """ Mostly <TODO>, link to ROS """

    def __init__(self, origin):
        self.waypoints = list(range(14))
        self.hub = origin.replace(" ", "")
        self.last_location = self.hub
        self.target = None
        self.in_transit = False

    def go_to(self, location):
        """ Send command to navigate to given WayPoint """
        self.target = location.replace(" ", "")
        print "[NAV] Going to", self.target
        # Send goal to action server
        try:
            reply = subprocess.check_output(
                ["rosrun", "topological_navigation", "nav_client.py", self.target])
        except:
            reply = "False"
        # Parse result
        if 'True' in reply:
            self.last_location = self.target
            self.target = None
            print "[NAV] Arrived at", self.target
        else:
            print "[NAV] ERROR: Couldn't reach", self.target
        self.in_transit = False
        print self.last_location

    def go_to_hub(self):
        """ Send command to go to Hub """
        self.go_to(self.hub)

    def current_location(self):
        """ Return current location as Pose/WayPoint """
        location = "Lost"
        if not self.in_transit:
            location = self.last_location
        else:
            location = "Between {0} and {1}".format(self.last_location, self.target)
        return location
