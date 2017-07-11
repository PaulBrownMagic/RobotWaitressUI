#!/usr/bin/env python
import actionlib
import json
import os
import time
import roslib
import rospy
import sys
import topological_navigation.msg
from flask import Flask, redirect, render_template, request, session
from orders import Orders

app = Flask(__name__)

HUB = 'WayPoint1'
PIN = "1111"  # Not so secure Login password.
TWITTER = True  # Display link to twitter or not.

# User pages, anyone can view.


@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    return render_template("home.html", title="LUCIE | Menu", menu=menu, twitter=TWITTER)


@app.route("/order", methods=["GET", "POST"])
def order_page():
    """ Show when an order is received, take to the hub."""
    if request.method == "POST":
        orders.add(request.form)
        return_to_hub()
    elif orders.empty():
        return redirect("/all_orders", code=302)
    return render_template("order.html", title="LUCIE | Order", order=orders.last_order())


@app.route("/deliver/<orderId>")
def deliver_page(orderId):
    """ Show when making a delivery, start timeout at WayPoint location """
    navigation.go_to(orders.orders[orderId]['location'])
    return render_template("delivery.html", title="LUCIE | Delivery", order=orders.orders[orderId])


@app.route("/twitter")
def twitter_page():
    """ Show LUCIE's twitter feed, run with Twitter 'selfie' program. """
    return render_template("twitter.html", title="LUCIE | Twitter")


# Admin/Staff only pages
@app.route('/login', methods=['GET', 'POST'])
def login_page():
    """ Login User, requires a PIN code, grants admin privileges """
    error_msg = None
    if request.method == 'POST':  # Form has been submitted
        if request.form['password'] == PIN:
            session['logged_in'] = True
            return redirect("/", code=302)  # Logged in, send to home
        else:
            error_msg = "Incorrect PIN"  # Not logged in
    return render_template("login.html", title="LUCIE | Login", error=error_msg)


@app.route("/all_orders")
def all_orders_page():
    """ Give an overview of all orders placed, allows admin to load past orders """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    else:
        return render_template("all_orders.html", title="LUCIE | All Orders", orders=sorted(orders.orders.items()))


@app.route("/order/<orderId>")
def order_id_page(orderId):
    """ View an order given its ID (timestamp) """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    try:
        order = orders.orders[orderId]  # Order exists
    except KeyError:
        # Order doesn't exist, other errors will still raise
        return redirect("/all_orders", code=302)
    return render_template("order.html", title="LUCIE | Order", order=order, orderID=orderId)


@app.route("/navigation")
def go_to_page():
    """ Allows admin to tell LUCIE to go to a WayPoint """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    return render_template("go_to.html", title="LUCIE | Navigation", waypoints=navigation.waypoints)


# Non-User URLs: Handling data and navigation, hence all set to POST
@app.route("/cancel_order", methods=["POST"])
def cancel_order():
    """ Set order status to Cancelled """
    orders.cancel_order(request.form['orderId'])
    return redirect("/", code=302)  # Redirect back home as per user intention.


@app.route("/order_complete", methods=["POST"])
def complete_order():
    """ Set order status to Complete, AJAX only call as no clear user intent."""
    orders.complete_order(request.form['orderId'])
    return ""  # Must return a string for Flask


@app.route("/return_to_hub", methods=["POST"])
def return_to_hub():
    """ Called via AJAX to make LUCIE return to the hub """
    navigation.go_to_hub()  # rospy.loginfo()
    return ""  # Must return a string for Flask


@app.route("/go_to", methods=["POST"])
def go_to():
    """ Send LUCIE to a given destination, called in "/navigation" page """
    navigation.go_to(request.form['destination'])
    return redirect("/", code=302)


class Navigation(object):
    
    def __init__(self, origin):
        self.waypoints = list(range(14))
        self.hub = origin.replace(" ", "")
        self.last_location = self.hub
        self.target = None
        self.in_transit = False
        # Rospy
        rospy.on_shutdown(self.clear_goals)
        self.client = actionlib.SimpleActionClient('topological_navigation',
                                                   topological_navigation.msg.GotoNodeAction)
        self.client.wait_for_server()
        rospy.loginfo("[NAV] ... Init done")

    def go_to(self, location):
        """ Send command to navigate to given WayPoint """
        navgoal = topological_navigation.msg.GotoNodeGoal()
        self.target = location.replace(" ", "")
        navgoal.target = self.target
        print "Going to", self.target
        # Send goal to action server
        #reply = subprocess.check_output(["rosrun", "topological_navigation", "nav_client.py", self.target])
        self.client.send_goal(navgoal)
        # Parse result
        self.client.wait_for_result()
        print self.client.get_result()
        self.in_transit = False

    def go_to_hub(self):
        """ Send command to go to Hub """
        self.go_to(self.hub)

    def current_location(self):
        """ Return current location as Pose/WayPoint """
        location = "Lost"
        if not self.in_transit:
            location = self.last_location
        else:
            location = "Travelling from {0} to {1}".format(self.last_location, self.target)
        return location

    def clear_goals(self):
        self.client.cancel_all_goals()


if __name__ == "__main__":
    # Load in the menu
    try:
        menu_json = os.path.join(roslib.packages.get_pkg_dir('waitress_ui'), "scripts/menu.json")
    except:
        menu_json = "menu.json"
    with open(menu_json) as menu_file:
        menu = json.load(menu_file)['items']
    # Setup orders and navigation interfaces
    rospy.init_node('waitress_nav')
    navigation = Navigation(HUB)
    orders = Orders(navigation)
    # Run the app
    app.secret_key = os.urandom(12)  # For sessions, different on each run
    app.run(debug=True, host='0.0.0.0', port=os.environ.get(
        "PORT", 5000), processes=1)  # Debug only
    # app.run(host='0.0.0.0', port=os.environ.get("PORT", 5000), processes=1)  # Production

    
