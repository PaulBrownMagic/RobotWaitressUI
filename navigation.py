from multiprocessing import Array, Value

import actionlib
import rospy
import topological_navigation.msg


class Navigation(object):

    def __init__(self, origin, num_of_waypoints):
        self.waypoints = list(range(1, num_of_waypoints + 1))
        self.hub = origin.replace(" ", "")
        self.last_location = Array('c', "WayPoint999")  # Shared memory between processes
        self.target = Array('c', "WayPoint999")  # C-Types t/f Character array,
        self.in_transit = Value('i', 0)  # and 0 for False. Set to WayPoint999 for length of Array
        self.last_location.value = self.hub
        self.target.value = "None"
        # Rospy
        rospy.on_shutdown(self.clear_goals)
        self.client = actionlib.SimpleActionClient('topological_navigation',
                                                   topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        print "[NAV] ... Init done"

    def go_to(self, location):
        """ Send command to navigate to given WayPoint """
        self.in_transit.value = 1
        navgoal = topological_navigation.msg.GotoNodeGoal()
        self.target.value = location.replace(" ", "")
        navgoal.target = self.target.value
        print "[NAV] Going to", self.target.value
        # Send goal to action server
        self.client.send_goal(navgoal)
        # Parse result
        self.client.wait_for_result()
        result = self.client.get_result()
        if "Success" in result:
            self.last_location.value = self.target.value
            self.target.value = "None"
        self.in_transit.value = 0
        print "[NAV]", result

    def go_to_hub(self):
        """ Send command to go to Hub """
        self.go_to(self.hub)

    def current_location(self):
        """ Return current location as Pose/WayPoint """
        location = "Lost"
        if self.in_transit.value == 0:
            location = self.last_location.value
        else:
            location = self.target.value
        print "[NAV] I'm at", location
        return location

    def clear_goals(self):
        self.client.cancel_all_goals()
