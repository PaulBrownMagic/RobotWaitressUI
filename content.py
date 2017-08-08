from flask import render_template, session
from flask_socketio import Namespace, emit

from config import HUB, MENU, NUMBER_OF_WAYPOINTS, TWITTER


class ContentLoader(Namespace):

    def __init__(self, url_name, orders):
        super(Namespace, self). __init__(url_name)
        self.orders = orders

    def on_connect(self):
        """ Display the home content """
        self.on_home()

    def on_home(self):
        """ Display the home content """
        content = render_template("home.html",
                                  menu=MENU,
                                  twitter=TWITTER)
        emit('new_content', content)

    def on_home_nav(self, destination):
        content = render_template("home.html",
                                  destination=destination,
                                  menu=MENU,
                                  twitter=TWITTER)
        emit('new_content', content)

    def on_last_order(self):
        """ Display the order content """
        content = render_template("order.html",
                                  order=self.orders.query.all()[-1].read(),
                                  destination=HUB)
        emit('new_content', content)

    def on_deliver(self, order_time):
        """ Display the delivery content """
        order = self.orders.query.filter_by(timestamp=order_time).first().read()
        print "Deliver", order
        content = render_template("delivery.html",
                                  order=order,
                                  destination=order['location'])
        emit('new_content', content, broadcast=True)

    def on_twitter(self):
        """ Display the twitter content """
        content = render_template("twitter.html")
        emit('new_content', content)

    def on_all_orders(self):
        """ Admin: Display all orders view content """
        if not session or not session['logged_in']:
            content = render_template("home.html", menu=MENU, twitter=TWITTER)
        else:
            orders = [o.read() for o in self.orders.query.all()]
            content = render_template("all_orders.html",
                                      orders=orders)
        emit('new_content', content)

    def on_order(self, order_time):
        order = self.orders.query.filter_by(timestamp=order_time).first().read()
        content = render_template("order.html",
                                  order=order)
        emit('new_content', content)

    def on_navigation(self):
        """ Admin: Display navigation content """
        if not session or not session['logged_in']:
            content = render_template("home.html", menu=MENU, twitter=TWITTER)
        else:
            content = render_template("go_to.html",
                                      waypoints=NUMBER_OF_WAYPOINTS)
            emit('new_content', content)
