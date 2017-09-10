"""Interface for Order logging and persistence."""
import json
from datetime import datetime

from flask_socketio import Namespace, emit

import rospy
import waitress_gui

orders = waitress_gui.ordersdb


class Orders(orders.Model):
    """Database model for an Order."""

    id = orders.Column(orders.Integer, primary_key=True)
    items = orders.Column(orders.Text)
    location = orders.Column(orders.String(16))
    timestamp = orders.Column(orders.DateTime, default=datetime.utcnow)
    status = orders.Column(orders.String(16), default="Open")

    def __init__(self, items):
        """Create an instance of an order record."""
        super(Orders, self).__init__(items=json.dumps(items),
                                     location=waitress_gui.current_location[0],
                                     status="Open",
                                     timestamp=datetime.utcnow())
        self.log_order("RECEIVED")

    def __repr__(self):
        """Provide a readable string representation of an order."""
        return 'Order for {}, status is "{}"'.format(self.location,
                                                     self.status)

    def read_items(self):
        """Convert times from JSON string to Python dict."""
        return json.loads(self.items)

    def read(self):
        """Convert record into Python format for easy manipulation."""
        result = dict(id=self.id,
                      location=self.location,
                      timestamp=self.timestamp,
                      status=self.status,
                      items=self.read_items())
        return result

    def cancel(self):
        """Update record status to 'Cancelled'."""
        self.status = 'Cancelled'
        self.log_order("CANCELLED")

    def complete(self):
        """Update record status to 'Complete'."""
        self.status = 'Complete'
        self.log_order("COMPLETE")

    def log_order(self, description):
        """Log an order."""
        items = " ".join(["({}, {}), ".format(k.decode(), v)
                          for k, v in self.read_items().items()])
        items = "[" + items.strip(" ,") + "]"
        info = "[WAITRESS ORDER {}] time= {}, \
                                    location= {}, \
                                    status= {}, \
                                    items= {}".format(description,
                                                      self.timestamp,
                                                      self.location,
                                                      self.status,
                                                      items)
        rospy.loginfo(info)


class Opinion(orders.Model):
    """Database model for an Opinion, used for survey."""

    id = orders.Column(orders.Integer, primary_key=True)
    a = orders.Column(orders.Boolean)
    b = orders.Column(orders.Boolean)
    c = orders.Column(orders.Boolean)
    d = orders.Column(orders.Boolean)
    e = orders.Column(orders.Boolean)
    comment = orders.Column(orders.Text)

    def __repr__(self):
        """Provide a string representation of an opinion."""
        return "[pk:{}. a:{}, b:{}, c:{}, d:{}, e:{}, comment:{}], ".format(
            self.id,
            self.a,
            self.b,
            self.c,
            self.d,
            self.e,
            self.comment)


class OrdersWS(Namespace):
    """Interface between UI, via WebSocket, and Orders database."""

    def __init__(self, url_name):
        """Establish itself as a Namespace for WebSockets."""
        super(Namespace, self).__init__(url_name)

    def on_add(self, order_arr):
        """Add a new Order record to the database."""
        order = {}
        for item in order_arr:
            order[item['name']] = item['value']
        order = Orders(order)
        orders.session.add(order)
        orders.session.commit()

    def update_status(self, order_id, status):
        """Change the status of a given order to a given status."""
        order = Orders.query.filter_by(timestamp=order_id).first()
        order.status = status
        order.log_order(status)
        orders.session.commit()

    def on_cancel(self, order_id):
        """Set a given order's status to 'Cancelled'."""
        self.update_status(order_id, "Cancelled")

    def on_complete(self, order_id):
        """Set a given order's status to 'Complete'."""
        self.update_status(order_id, "Complete")

    def on_opinion_submitted(self, opinion_arr):
        """Add a new Opinion record to the database."""
        opinions = []
        for inpt in opinion_arr:
            if inpt['name'] == "opinion":
                opinions.append(inpt['value'])
            elif inpt['value'] == '':
                comment = None
            else:
                comment = inpt['value']
        opinion = Opinion(a="a" in opinions,
                          b="b" in opinions,
                          c="c" in opinions,
                          d="d" in opinions,
                          e="e" in opinions,
                          comment=comment)
        orders.session.add(opinion)
        orders.session.commit()
        emit("got_opinion")


"""Uncomment as appropriate."""
# orders.drop_all()
orders.create_all()
