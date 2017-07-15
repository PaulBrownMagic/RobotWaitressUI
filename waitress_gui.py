#!/usr/bin/env python
import json
import os
from random import choice

from flask import Flask, redirect, render_template, request, session
from flask_socketio import SocketIO

import roslib
import rospy
from iwebsocket import SocketHelper, background_thread
from navigation import Navigation
from orders import Orders

# Constants MOVE TO CONFIG.py along with MENU
HUB = 'WayPoint1'  # Kitchen/Food source
NUMBER_OF_WAYPOINTS = 14
ONE_MACHINE = True  # Only working with LUCIE, no login for admin features
PIN = "1111"  # Not so secure Login password.
WONDERING_MODE = True  # Randomly visit WayPoints when looking for an order
TWITTER = True  # Display link to twitter or not. Use with selfie program.

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None
# Load in the menu
app = Flask(__name__)
app.secret_key = os.urandom(12)  # For sessions, different on each run
socketio = SocketIO(app, async_mode=async_mode)
socketio.on_namespace(SocketHelper('/io', socketio=socketio))


# User pages, anyone can view.
@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    if ONE_MACHINE:  # Auto-login, for convenience when using just LUCIE
        session['logged_in'] = True
    return render_template("home.html",
                           title="LUCIE | Menu",
                           menu=menu,
                           twitter=TWITTER,
                           async_mode=socketio.async_mode)


@app.route("/order", methods=["GET", "POST"])
def order_page():
    """ Displays last order.

    Show when an order is received, navigate to the hub.
    """
    if request.method == "POST":
        # Must log order before nav for correnct location
        orders.add(request.form, navigation.current_location())
    elif orders.empty():
        return redirect("/all_orders", code=302)
    return render_template("order.html",
                           title="LUCIE | Order",
                           order=orders.last_order(),
                           async_mode=socketio.async_mode)


@app.route("/deliver/<orderId>")
def deliver_page(orderId):
    """ Show when making a delivery, start timeout at WayPoint location """
    return render_template("delivery.html",
                           title="LUCIE | Delivery",
                           order=orders.orders[orderId],
                           async_mode=socketio.async_mode)


@app.route("/twitter")
def twitter_page():
    """ Show LUCIE's twitter feed, run with Twitter 'selfie' program. """
    return render_template("twitter.html",
                           title="LUCIE | Twitter",
                           async_mode=socketio.async_mode)


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
    return render_template("login.html",
                           title="LUCIE | Login",
                           error=error_msg,
                           async_mode=socketio.async_mode)


@app.route("/all_orders")
def all_orders_page():
    """ Give an overview of all orders placed, allows admin to load past orders """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    else:
        return render_template("all_orders.html",
                               title="LUCIE | All Orders",
                               orders=sorted(orders.orders.items()),
                               async_mode=socketio.async_mode)


@app.route("/order/<orderId>")
def order_id_page(orderId):
    """ View an order given its ID (timestamp)

    No navigation, unlike the "/order" route.
    """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    try:
        order = orders.orders[orderId]  # Order exists
    except KeyError:
        # Order doesn't exist, other errors will still raise
        return redirect("/all_orders", code=302)
    return render_template("order.html",
                           title="LUCIE | Order",
                           order=order,
                           orderID=orderId,
                           async_mode=socketio.async_mode)


@app.route("/navigation")
def go_to_page():
    """ Allows admin to tell LUCIE to go to a WayPoint """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    return render_template("go_to.html",
                           title="LUCIE | Navigation",
                           waypoints=navigation.waypoints,
                           async_mode=socketio.async_mode)


# Non-User URLs: Handling data and navigation, hence all set to POST
@app.route("/cancel_order", methods=["POST"])
def cancel_order():
    """ Set order status to Cancelled """
    orders.cancel_order(request.form['orderId'])
    return redirect("/", code=302)  # Redirect back home as per user intention.


@app.route("/order_complete", methods=["POST"])
def complete_order():
    """ Set order status to Complete."""
    # Currently Delivery Screen No Thanks needs to call random destination
    orders.complete_order(request.form['orderId'])
    return redirect("/", code=302)


@app.route("/return_to_hub", methods=["POST"])
def return_to_hub():
    """ Called via AJAX to make LUCIE return to the hub """
    # Run navigation in parallel to not delay UI
    # navigation.go_to_hub()
    return ""  # Must return a string for Flask


@app.route("/go_to", methods=["POST"])
def go_to():
    """ Send LUCIE to a given destination, called in "/navigation" page """
    # navigation.go_to(request.form['destination'])
    return redirect("/", code=302)


@app.route("/go_to_random", methods=["POST"])
def go_to_random():
    """ Send LUCIE to a random WayPoint

    Useful when looking for orders in an unintelligent manner
    """
    if WONDERING_MODE:
        destination = "WayPoint{}".format(choice(navigation.waypoints))
        navigation.go_to(destination)
    else:
        navigation.go_to_hub()
    return redirect("/", code=302)


# Run program
if __name__ == "__main__":
    try:
        menu_json = os.path.join(roslib.packages.get_pkg_dir('waitress_ui'), "scripts/menu.json")
    except:
        menu_json = "menu.json"
    with open(menu_json) as menu_file:
        menu = json.load(menu_file)['items']
    # Setup orders and navigation interfaces
    rospy.init_node('waitress_nav')
    navigation = Navigation(HUB, NUMBER_OF_WAYPOINTS)
    orders = Orders()
    rospy.loginfo("[WAITRESS] UI Launched at http://0.0.0.0:5000")
    # Run the app
    socketio.run(app, debug=True)  # Debug only
    # app.run(host='0.0.0.0', port=os.environ.get("PORT", 5000), processes=1)  # Production
