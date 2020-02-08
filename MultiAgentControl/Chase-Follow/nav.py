# Library containing methods for navigation of drones to given waypoints

import json

class Nav:
    def __init__(self, waypoint_list_path):
        # Load waypoints for lead to follow and chase to receive
        self.waypoints = json.load(open(waypoint_list_path, 'rb'))


if __name__ == "__main__":
    new_nav = nav('waypoints.json')
    for waypoint in new_nav.waypoints["points"]:
        print(waypoint)
    for i, waypoint in enumerate(new_nav.waypoints["points"]):
        print(i, waypoint)