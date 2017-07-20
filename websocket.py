from random import choice

from flask import request
from flask_socketio import Namespace, disconnect, emit


class SocketHelper(Namespace):

    thread = None  # Used to call background_task used to test help functions

    def __init__(self, urlname, socketio=None, navigation=None, orders=None):
        super(Namespace, self).__init__(urlname)
        self.socketio = socketio
        self.navigation = navigation
        self.orders = orders

    # Incoming messages
    def on_return_to_hub(self):
        """ Ask Nav to return to hub """
        print("[WS] Return to Hub")
        self.navigation.go_to_hub()

    def on_go_to(self, message):
        """ Ask Nav to go to WayPoint """
        print("[WS] Go To {}".format(message['destination']))
        self.navigation.go_to(message['destination'])

    def on_choose_destination(self):
        """ Ask Nav to follow it's go_to_random() protocol """
        print("[WS] Go To Random")
        self.navigation.go_to_random()

    def on_order_complete(self, message):
        """ Ask Orders to set order status to Complete """
        print("[WS] Order {} Complete".format(message['orderId']))
        self.orders.complete_order(message['orderId'])

    def on_disconnect_request(self):
        """ disconnect user from socket """
        disconnect()

    def on_connect(self):
        """ Test Helper messages """
        # if self.thread is None:
        #    self.thread = self.socketio.start_background_task(
        #        target=background_thread, socketio=self.socketio, helper=self.helper)
        print('Client connected', request.sid)

    def on_disconnect(self):
        print('Client disconnected', request.sid)
