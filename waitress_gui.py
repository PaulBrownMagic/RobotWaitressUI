#!/usr/bin/env python
import json
import os
import time
import roslib
from flask import Flask, redirect, render_template, request, session
from navigation import Navigation

app = Flask(__name__)

PIN = "1111"  # Not so secure Login password.
TWITTER = True  # Display link to twitter or not.


class Orders(object):
    """ Class to handle and log orders, uses a dictionary internally"""

    def __init__(self):
        self.orders = {}  # Holds the log of orders
        self.last_order_id = None  # Utility to remember last order key

    def add(self, order):
        """ Add an item to the orders record """
        timestamp = time.strftime("%H:%M:%S", time.localtime())  # Key as time
        item = {'timestamp': timestamp, 'location': navigation.current_location(),
                'status': "Open", 'items': order}  # An "order" as a record
        self.last_order_id = timestamp
        self.orders[self.last_order_id] = item

    def last_order(self):
        """ Get the last order record """
        return self.orders[self.last_order_id]

    def cancel_order(self, index):
        """ Set a order's status to Cancelled """
        self.orders[index]['status'] = "Cancelled"

    def cancel_last_order(self):
        """ Set the last order's status to Cancelled """
        self.cancel_order(self.last_order_id)

    def complete_order(self, index):
        """ Set an order's status to Complete """
        self.orders[index]['status'] = "Complete"

    def complete_last_order(self):
        """ Set the last order's status to Complete """
        self.complete_order(self.last_order_id)

    def empty(self):
        """ Check if there are any orders """
        return self.last_order_id is None


# User pages, anyone can view.
@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    return render_template("home.html", title="LUCIE | Menu", menu=menu, twitter=TWITTER)


@app.route("/order", methods=["GET", "POST"])
def order_page():
    """ Show when an order is received, take to the hub."""
    if request.method == "POST":
        return_to_hub()
        orders.add(request.form)
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


# Non-User urls: Handling data and navigation, hence all set to POST
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


if __name__ == "__main__":
    # Load in the menu
    menu_json = os.path.join(roslib.packages.get_pkg_dir('waitress_ui'), "scripts/menu.json")
    with open(menu_json) as menu_file:
        menu = json.load(menu_file)['items']
    # Setup orders and navigation interfaces
    orders = Orders()
    navigation = Navigation('WayPoint 1')
    # Run the app
    app.secret_key = os.urandom(12)  # For sessions, different on each run
    app.run(debug=True, host='0.0.0.0', port=os.environ.get("PORT", 5000), processes=1)  # Debug only
    # app.run(host='0.0.0.0', port=os.environ.get("PORT", 5000), processes=1)  # Production
