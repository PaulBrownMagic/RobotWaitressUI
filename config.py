"""Configurable constants for Application."""

# MENU['name'] will be displayed in HTML
# MENU['image'] is expected in static/images, square images look best.
MENU = [{"name": "Red Sweet", "image": "red_sweet.jpg"},
        {"name": "Green Sweet", "image": "green_sweet.jpg"},
        {"name": "Blue Sweet", "image": "blue_sweet.jpg"}
        ]
HUB = 'WayPoint1'  # Kitchen/Food source
NUMBER_OF_WAYPOINTS = 20
ONE_MACHINE = True  # Only working with LUCIE, no login for admin features
PIN = "1111"  # Not so secure Login password.
# NAVIGATING_MODE Options: "HUB" "RANDOM" "PATROL" "RANDOM_PATROL"
NAVIGATING_MODE = "PATROL"
TWITTER = True  # Display link to twitter or not. Use with selfie program.
