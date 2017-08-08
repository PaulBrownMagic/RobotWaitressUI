#!/usr/bin/env python
import json
import os
from datetime import datetime
from random import choice

from flask import Flask, redirect, render_template, request, session
from flask_socketio import Namespace, SocketIO, emit
from flask_sqlalchemy import SQLAlchemy

import actionlib
import rospy
import topological_navigation.msg
from config import (HUB, MENU, NUMBER_OF_WAYPOINTS, ONE_MACHINE, PIN, TWITTER,
                    WONDERING_MODE)
from content import ContentLoader
import orders
from navigation import Navigator

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None
# Create the app
app = Flask(__name__)
app.secret_key = os.urandom(12)  # For sessions, different on each run
rospy.init_node('waitress_nav')
# Setup socketio for websocket and other helper classes
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:////tmp/test.db'
ordersdb = SQLAlchemy(app)
socketio = SocketIO(app, async_mode=async_mode)
current_location = [HUB]  # Mutable data type required


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


@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    if ONE_MACHINE:  # Auto-login, for convenience when using just LUCIE
        session['logged_in'] = True
    return render_template("base.html", title="LUCIE | Menu", async_mode=socketio.async_mode)


# Run program
if __name__ == "__main__":
    socketio.on_namespace(Navigator('/nav', HUB))
    socketio.on_namespace(ContentLoader('/content', orders.Orders))
    socketio.on_namespace(orders.OrdersWS('/orders', ordersdb))
    # Setup orders and navigation interfaces
    ordersdb.drop_all()
    ordersdb.create_all()
    rospy.loginfo("[WAITRESS] UI Launched at http://0.0.0.0:5000")
    # Run the app
    socketio.run(app, host='0.0.0.0', port=os.environ.get("PORT", 5000), debug=True)  # Debug only
    # socketio.run(host='0.0.0.0', port=os.environ.get("PORT", 5000))  # Production
