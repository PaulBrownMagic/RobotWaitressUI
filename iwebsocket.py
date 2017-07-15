from flask import request
from flask_socketio import Namespace, SocketIO, disconnect, emit


class SocketHelper(Namespace):

    thread = None

    def __init__(self, urlname, socketio=None, navigation=None):
        super(Namespace, self).__init__(urlname)
        self.socketio = socketio
        self.navigation = navigation

    def on_return_to_hub(self):
        print("[WS] Return to Hub")
        self.navigation.go_to_hub()
        emit('my_response', {'data': "Return to Hub"})

    def on_go_to(self, message):
        print("[WS] Go To {}".format(message['destination']))
        self.navigation.go_to(message['destination'])
        emit('my_response', {'data': "Going to {}".format(message['destination'])})

    def on_choose_destination(self):
        print("[WS] Go To Random")
        self.navigation.go_to_random()
        emit('my_response', {'data': "Going to Random"})

    def on_order_complete(self, message):
        print("[WS] Order {} Complete".format(message['orderId']))
        emit('my_response', {'data': "Order {} Complete".format(message['orderId'])})

    def on_my_event(self, message):
        emit('my_response', {'data': message['data']})

    def on_my_broadcast_event(self, message):
        emit('my_response', {'data': message['data']}, broadcast=True)

    def on_disconnect_request(self):
        emit('my_response', {'data': 'Disconnected!'})
        disconnect()

    def on_my_ping(self):
        emit('my_pong')

    def on_connect(self):
        if self.thread is None:
            self.thread = self.socketio.start_background_task(
                target=background_thread, socketio=self.socketio)
        emit('my_response', {'data': 'Connected'})

    def on_disconnect(self):
        print('Client disconnected', request.sid)


def background_thread(socketio):
    """Example of how to send server generated events to clients."""
    count = 0
    while True:
        socketio.sleep(30)
        count += 1
        socketio.emit('my_response',
                      {'data': 'Server generated event'},
                      namespace='/io')
