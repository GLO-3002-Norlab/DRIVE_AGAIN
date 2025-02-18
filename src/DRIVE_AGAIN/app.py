from DRIVE_AGAIN.server import Server
from threading import Thread
from DRIVE_AGAIN.sim import Sim
import matplotlib.pyplot as plt


def start_drive_cb():
    print("starting drive")


def start_geofence_cb():
    print("starting geofence")


def stop_geofence_cb():
    print("stopping geofence")


def main():
    plt.switch_backend("Agg")
    server = Server(start_drive_cb, start_geofence_cb, stop_geofence_cb)
    sim = Sim(server)

    sim_thread = Thread(target=sim.run)
    sim_thread.start()

    server.run()
    sim_thread.join()


if __name__ == "__main__":
    main()
