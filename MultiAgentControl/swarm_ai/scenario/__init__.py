# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# 
# Description: Scenario class to hold all details of the scenario that will
#              be used for the actual simulation.
# ===============================================================
from scenario.maps import Rendezvous

class Scenario():

    def __init__(self, weather=[], id=1):
        """
            Currently, we are overwriting the __init__ function for
            the Drone class when we instantiate this model. This may
            change in the future.
        """
        self.id = id
        self.current_scenarios = {"1": Rendezvous()}
        # In the future, this will need to pull all of the information
        # about Rendezvous from a central place so we can easily update
        # the attributes as we support more work.
        self.selection = self.current_scenarios[str(id)]
        self.map_boundary = {"+X": self.selection.pos_x_boundary,
                             "-X": self.selection.neg_x_boundary,
                             "+Y": self.selection.pos_y_boundary,
                             "-Y": self.selection.neg_y_boundary}
        self.weather = [effect for effect in weather if len(weather) > 0]