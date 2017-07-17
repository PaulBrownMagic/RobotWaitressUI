from random import choice

import actionlib
import rospy
import topological_navigation.msg
from config import WONDERING_MODE


class Navigation(object):

    def __init__(self, origin, num_of_waypoints):
        self.waypoints = list(range(1, num_of_waypoints + 1))  # List of WayPoint numbers
        self.hub = origin.replace(" ", "")  # From CONFIG, allows setting WayPoint for HUB
        self.last_location = self.hub  # Assume LUCIE starts at the HUB
        self.target = None  # Where LUCIE is heading
        self.in_transit = False  # Track when LUCIE is in motion
        # Rospy
        rospy.on_shutdown(self.clear_goals)
        self.client = actionlib.SimpleActionClient('topological_navigation',
                                                   topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        print "[NAV] ... Init done"

    def go_to(self, location):
        """ Send command to navigate to given WayPoint """
        self.in_transit = True
        navgoal = topological_navigation.msg.GotoNodeGoal()
        self.target = location.replace(" ", "")
        navgoal.target = self.target
        print "[NAV] Going to", self.target
        # Send goal to action server
        self.client.send_goal(navgoal)
        # Parse result
        self.client.wait_for_result()
        result = self.client.get_result()
        if result is None:
            self.clear_goals()
        elif "True" in result:
            self.last_location = self.target
            self.target = None
        self.in_transit = False
        print "[NAV]", result

    def go_to_hub(self):
        """ Send command to go to Hub """
        self.go_to(self.hub)

    def go_to_random(self):
        """ Send LUCIE to a random WayPoint

        Useful when looking for orders in an unintelligent manner
        """
        if WONDERING_MODE:
            destination = "WayPoint{}".format(choice(self.waypoints))
            self.go_to(destination)
        else:
            self.go_to_hub()

    def current_location(self):
        """ Return current location as Pose/WayPoint """
        if not self.in_transit:
            location = self.last_location
        else:
            location = self.target
        return location

    def clear_goals(self):
        self.client.cancel_all_goals()
