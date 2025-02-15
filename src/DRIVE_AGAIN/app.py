from threading import Thread

import matplotlib.pyplot as plt
from sim import Sim

from DRIVE_AGAIN.server import Server


def start_geofence_cb():
    print("starting geofence")


def stop_geofence_cb():
    print("stopping geofence")


def start_drive_cb():
    print("starting drive")


def stop_drive_cb():
    print("stopping drive")


def main():
    plt.switch_backend("Agg")
    server = Server(start_geofence_cb, stop_geofence_cb, start_drive_cb, stop_drive_cb)

    sim = Sim(server)

    sim_thread = Thread(target=sim.run)
    sim_thread.start()

    server.run()
    sim_thread.join()


if __name__ == "__main__":
    main()
