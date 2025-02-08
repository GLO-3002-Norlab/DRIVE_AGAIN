from DRIVE_AGAIN.server import Server
from threading import Thread
from sim import Sim
import sys


def main():
    is_teleop = True
    if len(sys.argv) > 1:
        if sys.argv[1] == "drive":
            is_teleop = False

    server = Server()
    sim = Sim(server, is_teleop)

    sim_thread = Thread(target=sim.run)
    sim_thread.start()

    server.run()
    sim_thread.join()


if __name__ == "__main__":
    main()
