"""Load content into webpage via WebSocket using Jinja2 Templates."""

import json

from flask import render_template, session
from flask_socketio import Namespace, emit

import rospy
from config import HUB, MENU, NUMBER_OF_WAYPOINTS, ONE_MACHINE, TWITTER


class ContentLoader(Namespace):
    """Load HTML into single page application.

    Accepts requests via websockets, renders the corresponding html,
    and returns it via websocket. JavaScript is used to insert the html
    into the page (my_scripts.js:contentsocket.on('new_content')).
    """

    def __init__(self, url_name, orders):
        """Establish itself as a Namespace for WebSockets."""
        super(Namespace, self). __init__(url_name)
        self.orders = orders  # Database containing orders.

    def on_connect(self):
        """Log connection and display the home content."""
        rospy.loginfo("Client connected to Websocket")
        print('Client connected')
        self.on_home()

    def on_home(self):
        """Display the home content.

        URL: '/'
        """
        content = render_template("home.html",
                                  menu=MENU,
                                  twitter=TWITTER
                                  )
        emit('new_content', content)

    def on_home_nav(self, destination):
        """Display the home content with Timeout.

        URL: '/'
        """
        content = render_template("home.html",
                                  destination=destination,
                                  menu=MENU,
                                  twitter=TWITTER,
                                  timeout=30)
        emit('new_content', content)
        if not ONE_MACHINE:
            emit('cancel_timeout_on_admin')

    def on_last_order(self):
        """Display the order content.

        URL: '/order'
        """
        order = self.orders.query.all()[-1].read()
        content = render_template("order.html",
                                  order=order,
                                  destination=HUB)
        emit('new_content', content)
        order_info = {"title": "New Order",
                      "text": "New order for {}".format(order['location'])}
        if not ONE_MACHINE:
            emit('info', order_info, broadcast=True)

    def on_deliver(self, time):
        """Display the delivery content.

        URL: '/deliver/<time>'
        """
        order = self.orders.query.filter_by(timestamp=time).first().read()
        print "Deliver", order
        content = render_template("delivery.html",
                                  order=order,
                                  destination=order['location'],
                                  timeout=60)
        emit('new_content', content, broadcast=True)
        if not ONE_MACHINE:
            emit('cancel_timeout_on_admin')

    def on_twitter(self):
        """Display the twitter content.

        URL: '/twitter'
        """
        content = render_template("twitter.html")
        emit('new_content', content)

    def on_all_orders(self):
        """Admin: Display all orders view content.

        URL: '/all_orders'
        """
        if not session or not session['logged_in']:
            content = render_template("home.html", menu=MENU, twitter=TWITTER)
        else:
            orders = [o.read() for o in self.orders.query.all()]
            content = render_template("all_orders.html",
                                      orders=orders)
        emit('new_content', content)

    def on_order(self, time):
        """Display the order content.

        URL: '/order/<time>'
        """
        order = self.orders.query.filter_by(timestamp=time).first().read()
        content = render_template("order.html",
                                  order=order)
        emit('new_content', content)

    def on_navigation(self):
        """Admin: Display navigation content.

        URL: '/navigation'
        """
        if not session or not session['logged_in']:
            content = render_template("home.html", menu=MENU, twitter=TWITTER)
        else:
            content = render_template("go_to.html",
                                      waypoints=NUMBER_OF_WAYPOINTS)
            emit('new_content', content)

    def on_disconnect(self):
        """Log disconnection."""
        rospy.loginfo("Client disconnected from Websocket")
        print('Client disconnected')
