# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: November 18th, 2020
#
# Description: Main class for collecting data during algorithm execution.
# ===============================================================

import psycopg2
from psycopg2.extras import execute_values
import traceback

class DB:

    def __init__(self, user='airsim_user', password='airsim'):
        self.db_name = 'swarm'
        self.user = user
        self.password = password
        self.connection = None
        self.data_structs = {"drone": """ INSERT INTO drones (sim_id, name, weight, autopilot) VALUES (%s, %s, %s, %s) """,
                             "simulation": """ INSERT INTO simulations (id, swarm_id) VALUES (%s, %s, %s) """,
                             "position": """ INSERT INTO positions (sim_id, drone_name, x_val, y_val, z_val, lat, lon, alt, vel_x_val, vel_y_val, vel_z_val, time_step) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) """,
                             "distances": """ INSERT INTO distances (sim_id, distance, time_step) VALUES (%s, %s, %s) """,
                             "cmd_pos": """ INSERT INTO commanded_positions (sim_id, drone_name, x_val, y_val, z_val, lat, lon, alt, vel_x_val, vel_y_val, vel_z_val, time_step) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) """,
                             "state_vector": """ INSERT INTO state_vector (sim_id, state_vector, time_step) VALUES (%s, %s, %s) """,
                             "metric": """ INSERT INTO metrics (sim_id, connectivity, safety_inter_agent, safety_obstacle, union_m, order_m, time_step) VALUES (%s, %s, %s, %s, %s, %s, %s) """,
                             "state_vectors": """ INSERT INTO state_vector (sim_id, state_vector, time_step) VALUES %s """,
                             "positions": """ INSERT INTO positions (sim_id, drone_name, x_val, y_val, z_val, lat, lon, alt, vel_x_val, vel_y_val, vel_z_val, time_step) VALUES %s """,
                             "cmd_poss": """ INSERT INTO commanded_positions (sim_id, drone_name, x_val, y_val, z_val, lat, lon, alt, vel_x_val, vel_y_val, vel_z_val, time_step) VALUES %s """,
                             "distance_matrices": """ INSERT INTO distances (sim_id, distance, time_step) VALUES %s """}

    def make_connection(self):
        try:
            self.connection = psycopg2.connect(host='localhost', database=self.db_name, user=self.user, password=self.password)
            return True
        except Exception:
            traceback.print_exc()
            return False

    def commit_result(self, sql_string, data):
        """
        Example of input into database:

            sql_string = INSERT INTO positions (drone_id, x_val, y_val, z_val, lat, lon, alt, velocity, time_step) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);

            Data must be a tuple and PSQL can interpret the type of variable to input based off of the Column name.
            data = (1, 2.0, 3.0, -4.0, 0042.42313, -0132.34314, 212.22, 2.5, datetime.datetime.utcnow())
        """
        try:
            with self.connection.cursor() as cursor:
                cursor.execute(sql_string, data)
            # Commit the transaction to the database. If this fails, it will only rollback the transcation
            # to the last commited transaction. This allows any errors that occur or if the system is shut
            # down unexpectedly to not affect the data entry process or corrput the database.
            self.connection.commit()
            return True
        except Exception:
            traceback.print_exc()
            return False

    def commit_many_results(self, sql_string, data):
        try:
            with self.connection.cursor() as cursor:
                execute_values(cursor, sql_string, data)
            self.connection.commit()
            return True
        except Exception:
            traceback.print_exc()
            return False


