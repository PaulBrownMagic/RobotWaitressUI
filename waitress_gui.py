#!/usr/bin/env python
import json
import os
from multiprocessing import Process
from random import choice

from flask import Flask, redirect, render_template, request, session

import roslib
import rospy
from navigation import Navigation
from orders import Orders

# Constants
HUB = 'WayPoint1'  # Kitchen/Food source
NUMBER_OF_WAYPOINTS = 14
ONE_MACHINE = True  # Only working with LUCIE, no login for admin features
PIN = "1111"  # Not so secure Login password.
WONDERING_MODE = True  # Randomly visit WayPoints when looking for an order
TWITTER = True  # Display link to twitter or not. Use with selfie program.
STRANDS_UI_URL = "127.0.0.1:5000"

# Make the app!
app = Flask(__name__)


@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    if ONE_MACHINE:  # Auto-login, for convenience when using just LUCIE
        session['logged_in'] = True
    return render_template("home.html", title="LUCIE | Menu", menu=menu, twitter=TWITTER)


@app.route("/navigating/hub")
def navigating_to_hub_page():
    return render_template("navigating.html",
                           title="LUCIE | Moving",
                           destination="the Hub",
                           next_page="/order",
                           helper_url="http://{}".format(STRANDS_UI_URL))


@app.route("/navigating/random")
def go_to_random():
    """ Send LUCIE to a random WayPoint

    Useful when looking for orders in an unintelligent manner
    """
    if WONDERING_MODE:
        destination = "WayPoint{}".format(choice(navigation.waypoints))
    else:
        destination = HUB  # This method will redirect to "/" on navigation complete instead of "/order"
    return redirect("/navigating/{}".format(destination), code=302)


@app.route("/navigating/<destination>")
def navigating_to_page(destination):
    return render_template("navigating.html",
                           title="LUCIE | Moving",
                           destination=destination,
                           next_page="/",
                           helper_url="http://{}".format(STRANDS_UI_URL))


@app.route("/order", methods=["POST"])
def place_order():
    orders.add(request.form, navigation.current_location())
    return render_template("order.html", title="LUCIE | Order", order=orders.last_order())


@app.route("/order")  # GET
def order_page():
    """ Displays last order.

    Show once navigated to the hub.
    """
    if orders.empty():
        return redirect("/all_orders", code=302)
    return render_template("order.html", title="LUCIE | Order", order=orders.last_order())


@app.route("/deliver/<orderId>")
def deliver_page(orderId):
    """ Show when making a delivery, start timeout at WayPoint location """
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


@app.route("/go_to_hub", methods=["POST"])
def return_to_hub():
    """ Called via AJAX to make LUCIE return to the hub """
    # Run navigation in parallel to not delay UI
    navigation.go_to_hub()
    return ""  # Must return a string for Flask


@app.route("/go_to", methods=["POST"])
def go_to():
    """ Send LUCIE to a given destination, called in "/navigation" page """
    navigation.go_to(request.form['destination'])
    return ""


# Run program
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
    navigation = Navigation(HUB, NUMBER_OF_WAYPOINTS)
    orders = Orders()
    rospy.loginfo("[WAITRESS] UI Launched")
    # Run the app
    app.secret_key = os.urandom(12)  # For sessions, different on each run
    app.run(debug=True, host='0.0.0.0', port=os.environ.get(
        "PORT", 5000), processes=1)  # Debug only
    # app.run(host='0.0.0.0', port=os.environ.get("PORT", 5000), processes=1)  # Production
