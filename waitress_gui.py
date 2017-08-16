#!/usr/bin/env python
import os

from flask import Flask, redirect, render_template, request, session
from flask_socketio import SocketIO
from flask_sqlalchemy import SQLAlchemy

import orders
import rospy
from config import HUB, ONE_MACHINE, PIN
from content import ContentLoader
from navigation import Navigator


async_mode = None  # Manually config: threading, eventlet or gevent. Else None
# Create the app
app = Flask(__name__)
app.secret_key = os.urandom(12)  # For sessions, different on each run
# Setup store
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:////tmp/test.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False  # Mute warning
ordersdb = SQLAlchemy(app)
# Setup socketio
socketio = SocketIO(app,
                    async_mode=async_mode,
                    ping_interval=60,
                    ping_timeout=60*15)
# Global in memory variable to track current location
current_location = [HUB]  # Mutable data type required for behaviour


@app.route('/login', methods=['GET', 'POST'])
def login_page():
    """Login User, requires a PIN code, grants admin privileges."""
    error_msg = None
    if request.method == 'POST':  # Form has been submitted
        if request.form['password'] == PIN:
            session['logged_in'] = True
            return redirect("/", code=302)  # Logged in, send to home
        else:
            error_msg = "Incorrect PIN"  # Not logged in
    return render_template("login.html",
                           title="LUCIE | Login",
                           error=error_msg)


@app.route("/")
def home_page():
    """Show the menu and allow a user to place an order."""
    if ONE_MACHINE:  # Auto-login, for convenience when using just LUCIE
        session['logged_in'] = True
    return render_template("base.html",
                           title="LUCIE | Menu",
                           async_mode=socketio.async_mode)


# Run program
if __name__ == "__main__":
    rospy.init_node('waitress_nav')
    socketio.on_namespace(Navigator('/nav', HUB))
    socketio.on_namespace(ContentLoader('/content', orders.Orders))
    socketio.on_namespace(orders.OrdersWS('/orders', ordersdb))
    rospy.loginfo("[WAITRESS] UI Launched at http://0.0.0.0:5000")
    # Run the app
    socketio.run(app,
                 host='0.0.0.0',
                 port=os.environ.get("PORT", 5000),
                 debug=True)  # Debug only
#    socketio.run(app,
#                 host='0.0.0.0',
#                 port=os.environ.get("PORT", 5000))  # Production
