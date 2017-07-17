#!/usr/bin/env python
import os

from flask import Flask, redirect, render_template, request, session
from flask_socketio import SocketIO

import rospy
from config import MENU, TWITTER, HUB, NUMBER_OF_WAYPOINTS, ONE_MACHINE, PIN
from help_screen import HelpScreen
from websocket import SocketHelper
from navigation import Navigation
from orders import Orders

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = "eventlet"
# Create the app
app = Flask(__name__)
app.secret_key = os.urandom(12)  # For sessions, different on each run
rospy.init_node('waitress_nav')
# Setup socketio for websocket and other helper classes
socketio = SocketIO(app, async_mode=async_mode)
helper = HelpScreen(socketio=socketio)  # Monitored Navigation Help
navigation = Navigation(HUB, NUMBER_OF_WAYPOINTS)  # Topological Navigation
orders = Orders()  # Order "model" interface
socketio.on_namespace(SocketHelper('/io',
                                   socketio=socketio,
                                   navigation=navigation,
                                   orders=orders,
                                   helper=helper))


@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    if ONE_MACHINE:  # Auto-login, for convenience when using just LUCIE
        session['logged_in'] = True
    return render_template("home.html",
                           title="LUCIE | Menu",
                           menu=MENU,
                           twitter=TWITTER,
                           async_mode=socketio.async_mode
                          )


@app.route("/order", methods=["GET", "POST"])
def order_page():
    """ Displays last order.

    Show when an order is received, navigates to the hub via websocket.
    """
    if request.method == "POST":
        orders.add(request.form, navigation.current_location())
    elif orders.empty():
        return redirect("/all_orders", code=302)
    return render_template("order.html",
                           title="LUCIE | Order",
                           order=orders.last_order(),
                           async_mode=socketio.async_mode
                          )


@app.route("/deliver/<orderId>")
def deliver_page(orderId):
    """ Show when making a delivery, start timeout at WayPoint location """
    return render_template("delivery.html",
                           title="LUCIE | Delivery",
                           order=orders.orders[orderId],
                           async_mode=socketio.async_mode
                          )


@app.route("/twitter")
def twitter_page():
    """ Show LUCIE's twitter feed, run with Twitter 'selfie' program. """
    return render_template("twitter.html",
                           title="LUCIE | Twitter",
                          )


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
                           async_mode=socketio.async_mode
                          )


@app.route("/all_orders")
def all_orders_page():
    """ Give an overview of all orders placed, allows admin to load past orders """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    else:
        return render_template("all_orders.html",
                               title="LUCIE | All Orders",
                               orders=sorted(orders.orders.items()),
                               async_mode=socketio.async_mode
                              )


@app.route("/order/<orderId>")
def order_id_page(orderId):
    """ View an order given its ID (timestamp)

    No navigation via websocket, unlike the "/order" route. Useful for
    admin viewing.
    """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    try:
        order = orders.orders[orderId]  # Order exists
    except KeyError: # Order doesn't exist, other errors will still raise
        return redirect("/all_orders", code=302)
    return render_template("order.html",
                           title="LUCIE | Order",
                           order=order,
                           orderID=orderId,
                           async_mode=socketio.async_mode
                          )


@app.route("/navigation")
def go_to_page():
    """ Allows admin to tell LUCIE to go to a WayPoint """
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    return render_template("go_to.html",
                           title="LUCIE | Navigation",
                           waypoints=navigation.waypoints,
                           async_mode=socketio.async_mode
                          )


# Non-User URLs: Handling orders, hence all set to POST
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


# Run program
if __name__ == "__main__":
    # Setup orders and navigation interfaces
    rospy.loginfo("[WAITRESS] UI Launched at http://0.0.0.0:5000")
    # Run the app
    socketio.run(app, host='0.0.0.0', port=os.environ.get("PORT", 5000), debug=True)  # Debug only
    # socketio.run(host='0.0.0.0', port=os.environ.get("PORT", 5000))  # Production
