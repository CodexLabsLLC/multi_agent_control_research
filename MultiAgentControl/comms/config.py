
# Created By: Tyler Fedrizzi
# Created On: 3/27/2020
#
# Load configuration parameters from a JSON file for easy conversion

import json
import utils.singleton

@Singleton
class Parameters:
    def __init__(self, file_path):
        with open(file_path, 'r') as file:
            params_list = json.loads(file.read())