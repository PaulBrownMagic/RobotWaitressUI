import sys

import actionlib
import rospy
import topological_navigation.msg


class Navigation(object):
    """ Mostly <TODO>, link to ROS """

    def __init__(self, origin):
        self.waypoints = list(range(14))
        self.hub = origin
        self.last_location = origin
        self.target = None
        self.in_transit = False

        rospy.on_shutdown(self.clear_goals)
        self.client = actionlib.SimpleActionClient('topological_navigation',
                                                   topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        rospy.loginfo("... Navigation Init done")

    def go_to(self, location):
        """ Send command to navigate to given WayPoint """
        self.target = location
        navgoal = topological_navigation.msg.GotoNodeGoal()
        print "Going to {}".format(self.target)
        navgoal.target = self.target
        # Send goal to action server
        self.in_transit = True
        self.client.send_goal(navgoal)
        # Wait for server to finish performing the action
        self.client.wait_for_result()
        self.in_transit = False
        self.last_location = self.target
        self.target = None
        result = self.client.get_result()
        print "RESULT: ", result

    def go_to_hub(self):
        """ Send command to go to Hub """
        self.go_to(self.hub)

    def current_location(self):
        """ Return current location as Pose/WayPoint """
        location = "Lost"
        if not self.in_transit:
            location = self.last_location
        else
            location = "Travelling from {} to {}".format(self.last_location, self.target)
        return location

    def clear_goals(self):
        self.client.cancel_all_goals()


rospy.init_node('waitress_nav')
