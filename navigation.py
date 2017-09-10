"""Interface between UI and /topological_navigation."""
from random import randint, shuffle

from flask_socketio import Namespace, emit

import actionlib
import rospy
import topological_navigation.msg
import waitress_gui
from config import NUMBER_OF_WAYPOINTS, NAVIGATING_MODE, HUB


class Navigator(Namespace):
    """Interface between UI, via Websocket, and /topological_navigation."""

    def __init__(self, url_name, origin):
        """Create SimpleActionClient and Namespace for WebSocket."""
        super(Namespace, self).__init__(url_name)
        self.hub = origin.replace(" ", "")  # From CONFIG
        self.last_location = self.hub  # Assume LUCIE starts at the HUB
        self.target = None  # Where LUCIE is heading
        self.in_transit = False  # Track when LUCIE is in motion
        # Rospy
        rospy.on_shutdown(self.clear_goals)
        msg = topological_navigation.msg.GotoNodeAction
        self.client = actionlib.SimpleActionClient('topological_navigation',
                                                   msg)
        self.client.wait_for_server()
        print "[NAV] ... Init done"

    def go_to(self, location):
        """Send command to navigate to given WayPoint."""
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
        elif "True" in str(result):
            self.last_location = self.target
            info = "[NAV] Success, current location: {}".format(self.target)
            rospy.loginfo(info)
            self.target = None
            waitress_gui.current_location[0] = self.last_location
            emit('success')
        self.in_transit = False
        print "[NAV]", result
        if "False" in str(result):
            self.go_to(self.hub)
            rospy.loginfo("[NAV] Failed: Return to Hub")

    def clear_goals(self):
        """Clear all current navigation goals from the client."""
        self.client.cancel_all_goals()

    # Incoming messages
    def on_go_to_hub(self):
        """Ask Nav to return to hub."""
        print("[NAV] Return to Hub")
        rospy.loginfo("[NAV] Requesting navigation to Hub ({})".format(HUB))
        self.go_to(self.hub)

    def on_go_to(self, destination):
        """Ask Nav to go to WayPoint."""
        print("[NAV] Go To {}".format(destination))
        rospy.loginfo("[NAV] Requesting navigation to {}".format(destination))
        if destination == 'choose':
            self.on_choose_destination()
        else:
            self.go_to(destination)

    def on_choose_destination(self):
        """Ask Nav to follow its go_to_random() protocol."""
        if NAVIGATING_MODE == "RANDOM":
            destination = "WayPoint{}".format(randint(1, NUMBER_OF_WAYPOINTS))
        elif NAVIGATING_MODE == "PATROL":
            destination = "WayPoint{}".format(self.increment_waypoint())
        elif NAVIGATING_MODE == "RANDOM_PATROL":
            destination = "WayPoint{}".format(self.not_visited_waypoint())
        else:  # Should be "HUB" but left as a catch all for safety
            destination = self.hub
        self.go_to(destination)

    def increment_waypoint(self):
        """Return the next WayPoint in a cycle through WayPoints."""
        current_wp = int(self.last_location.split('t')[-1])
        next_wp = current_wp + 1 if current_wp != NUMBER_OF_WAYPOINTS else 1
        if "WayPoint{}".format(next_wp) == self.hub:
            next_wp = next_wp + 1 if next_wp != NUMBER_OF_WAYPOINTS else 1
        return next_wp

    def not_visited_waypoint(self):
        """Return next WayPoint from a shuffled list of WayPoints."""
        try:
            return next(self.waypoints)
        except StopIteration:
            self.generate_waypoints()
            return next(self.waypoints)
        except AttributeError:
            self.generate_waypoints()
            return next(self.waypoints)

    def generate_waypoints(self):
        """Create a shuffled list of all WayPoints."""
        points = [p + 1 for p in range(NUMBER_OF_WAYPOINTS)]
        shuffle(points)
        self.waypoints = (p for p in points
                          if "WayPoint{}".format(p) != self.hub)

    def on_clear_goals(self):
        """User has disconnected, clear the goals."""
        rospy.loginfo("[NAV] All goals cleared")
        self.clear_goals()
