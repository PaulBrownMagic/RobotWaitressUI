import json
import os
import time

from flask import Flask, redirect, render_template, request, session

app = Flask(__name__)

PIN = "1111"


class Orders:

    def __init__(self):
        self.orders = {}
        self.last_order_id = None

    def add(self, order):
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        item = {'timestamp': timestamp, 'location': navigation.current_location(),
                'status': "Open", 'items': order}
        self.last_order_id = timestamp
        self.orders[self.last_order_id] = item

    def last_order(self):
        return self.orders[self.last_order_id]

    def cancel_order(self, index):
        self.orders[index]['status'] = "Cancelled"

    def cancel_last_order(self):
        self.cancel_order(self.last_order_id)

    def complete_order(self, index):
        self.orders[index]['status'] = "Complete"

    def complete_last_order(self):
        self.complete_order(self.last_order_id)

    def empty(self):
        return self.last_order_id is None


class Navigation:

    def __init__(self):
        self.waypoints = list(range(14))

    def go_to(self, location):
        print "Going to {}".format(location)

    def go_to_hub(self):
        self.go_to("Hub")

    def current_location(self):
        return "Lost"

# User pages, anyone can view.


@app.route("/")
def home_page():
    """ Show the menu and allow a user to place an order """
    return render_template("home.html", title="LUCIE | Menu", menu=menu)


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


# Admin/Staff only pages
@app.route('/login', methods=['GET', 'POST'])
def login_page():
    if request.method == 'POST':
        if request.form['password'] == PIN:
            session['logged_in'] = True
            return redirect("/", code=302)
        else:
            error = "Incorrect password"
    return render_template("login.html", title="LUCIE | Login")


@app.route("/all_orders")
def all_orders_page():
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    else:
        return render_template("all_orders.html", title="LUCIE | All Orders", orders=sorted(orders.orders.items()))


@app.route("/order/<orderId>")
def order_id_page(orderId):
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    try:
        order = orders.orders[orderId]
    except:
        return redirect("/all_orders", code=302)
    return render_template("order.html", title="LUCIE | Order", order=order, orderID=orderId)


@app.route("/go_to")
def go_to_page():
    if not session or not session['logged_in']:
        return redirect("/", code=302)
    return render_template("go_to.html", title="LUCIE | Navigation", waypoints=navigation.waypoints)


# Non-User urls: Handling data and navigation, hence all set to POST
@app.route("/cancel_order", methods=["POST"])
def cancel_order():
    orders.cancel_order(request.form['orderId'])
    return redirect("/", code=302)


@app.route("/order_complete", methods=["POST"])
def complete_order():
    orders.complete_order(request.form['orderId'])
    return ""  # Must return a string for Flask


@app.route("/return_to_hub", methods=["POST"])
def return_to_hub():
    """ Called via AJAX to make LUCIE return to the hub """
    navigation.go_to_hub()  # rospy.loginfo()
    return ""  # Must return a string for Flask


@app.route("/go_to", methods=["POST"])
def go_to():
    navigation.go_to(request.form['destination'])
    return redirect("/", code=302)  # Must return a string for Flask


if __name__ == "__main__":
    # Load in the menu
    with open("menu.json") as menu_file:
        menu = json.load(menu_file)['items']
    orders = Orders()
    navigation = Navigation()
    # Run the app
    app.secret_key = os.urandom(12)  # For sessions
    app.run(debug=True)
