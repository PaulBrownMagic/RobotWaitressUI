from random import randint
from flask_socketio import Namespace
import actionlib
import rospy
import topological_navigation.msg
from config import WONDERING_MODE, NUMBER_OF_WAYPOINTS
import waitress_gui


class Navigator(Namespace):

    def __init__(self, url_name, origin):
        super(Namespace, self).__init__(url_name)
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
        nav_goal = topological_navigation.msg.GotoNodeGoal()
        self.target = location.replace(" ", "")
        nav_goal.target = self.target
        print "[NAV] Going to", self.target
        # Send goal to action server
        self.client.send_goal(nav_goal)
        # Parse result
        self.client.wait_for_result()
        result = self.client.get_result()
        if result is None:
            self.clear_goals()
        elif "True" in result:
            self.last_location = self.target
            self.target = None
            waitress_gui.current_location[0] = self.last_location
        self.in_transit = False
        print "[NAV]", result
        if "False" in result:
            self.go_to(self.hub)

    def clear_goals(self):
        self.client.cancel_all_goals()

    # Incoming messages
    def on_go_to_hub(self):
        """ Ask Nav to return to hub """
        print("[NAV] Return to Hub")
        self.go_to(self.hub)

    def on_go_to(self, destination):
        """ Ask Nav to go to WayPoint """
        print("[NAV] Go To {}".format(destination))
        if destination == 'random':
            self.on_choose_destination()
        else:
            self.go_to(destination)

    def on_choose_destination(self):
        """ Ask Nav to follow it's go_to_random() protocol """
        if WONDERING_MODE:
            destination = "WayPoint{}".format(randint(1, NUMBER_OF_WAYPOINTS))
            self.go_to(destination)
        else:
            self.go_to(self.hub)

    def on_clear_goals(self):
        """ disconnect user from socket """
        self.clear_goals()

    def on_connect(self):
        print('Client connected')

    def on_disconnect(self):
        print('Client disconnected')
