import time


class Orders(object):
    """ Class to handle and log orders, uses a dictionary internally"""

    def __init__(self, navigation):
        self.orders = {}  # Holds the log of orders
        self.last_order_id = None  # Utility to remember last order key
        self.navigation = navigation

    def add(self, order):
        """ Add an item to the orders record """
        timestamp = time.strftime("%H:%M:%S", time.localtime())  # Key as time
        item = {'timestamp': timestamp, 'location': self.navigation.current_location(),
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
