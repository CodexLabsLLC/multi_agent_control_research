# distance_utils.py
# Created by: Tyler Fedrizzi
# Created On: 21 March 2020

import math

def haversine(lat1, lon1, lat2, lon2):
    # distance between latitudes 
    # and longitudes 
    dLat = (lat2 - lat1) * math.pi / 180.0
    dLon = (lon2 - lon1) * math.pi / 180.0
  
    # convert to radians 
    lat1 = (lat1) * math.pi / 180.0
    lat2 = (lat2) * math.pi / 180.0
  
    # apply formulae 
    a = (pow(math.sin(dLat / 2), 2) + 
         pow(math.sin(dLon / 2), 2) * 
             math.cos(lat1) * math.cos(lat2)); 
    rad = 6371 # kilometers
    c = 2 * math.asin(math.sqrt(a)) 
    return rad * c # kilometers