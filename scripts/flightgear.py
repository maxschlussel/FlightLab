'''
Show flight visualization in FlightGear
'''

import math
import numpy as np
import pandas as pd
import socket
import subprocess
import time

from flightgear_python.fg_if import FDMConnection


# FlightGear startup CMD
# fgfs --native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp --max-fps=30 --fdm=null --telnet=5400 --prop:/sim/rendering/quality-level=0 --prop:/sim/rendering/shaders/quality-level=0 --disable-sound --timeofday=noon --enable-freeze --disable-ai-models --prop:/sim/current-view/view-number=3 --lat=63.9797 --lon=--22.5862
def start_flightgear():
    cmd = [
        "fgfs",
        "--native-fdm=socket,out,30,localhost,5501,udp",
        "--native-fdm=socket,in,30,localhost,5502,udp",
        "--max-fps=30",
        "--fdm=null",
        "--telnet=5400",
        "--prop:/sim/rendering/quality-level=0",
        "--prop:/sim/rendering/shaders/quality-level=0",
        "--disable-sound",
        "--timeofday=noon",
        "--enable-freeze",
        "--disable-ai-models",
        "--prop:/sim/current-view/view-number=3",
        "--lat=63.9797",
        "--lon=--22.5862"
    ]
    print("Starting FlightGear...")
    return subprocess.Popen(cmd)


def wait_for_fg_ready(host="127.0.0.1", port=5400, timeout=120):
    """Wait until FlightGear reports it's initialized via Telnet property tree."""
    print("Waiting 20 seconds for FlightGear to initialize...")
    time.sleep(40)
    return True

    # start = time.time()
    # while time.time() - start < timeout:
    #     try:
    #         with socket.create_connection((host, port), timeout=2) as s:
    #             # FG telnet is line-based, ask for a property
    #             s.sendall(b"get /sim/version/flightgear\n")
    #             resp = s.recv(1024).decode("utf-8", errors="ignore")
    #             print(resp)
    #             if resp:   # if you get anything back, FG is ready
    #                 print("FlightGear is responding!")
    #                 return True
    #             if "1" in resp:
    #                 print("FlightGear reports FDM initialized.")
    #                 return True
    #     except Exception as e:
    #         print("Still waiting for FG...", e)
    #         time.sleep(2)

    # return False


def ned_to_latlon(lat0, lon0, ned_n, ned_e, ned_d):
    """
    Converts a position from North-East-Down (NED) coordinates to
    geodetic latitude, longitude, and altitude, using a simple flat-earth model.

    Args:
        lat0 (float): Initial latitude in radians (origin of the NED frame).
        lon0 (float): Initial longitude in radians (origin of the NED frame).
        ned_n (float): North position in meters.
        ned_e (float): East position in meters.
        ned_d (float): Down position in meters.

    Returns:
        tuple: (lat_rad, lon_rad, alt_m)
            lat_rad (float): New latitude in radians.
            lon_rad (float): New longitude in radians.
            alt_m (float): New altitude in meters (positive up).
    """
    R_earth = 6378137.0  # [m]

    lat_rad = lat0 + (ned_n / R_earth)
    lon_rad = lon0 + (ned_e / (R_earth * math.cos(lat_rad)))
    alt_m = -ned_d

    return (lat_rad, lon_rad, alt_m)

    
def fdm_callback(fdm_data, event_pipe):
    if event_pipe.child_poll():
        data, = event_pipe.child_recv()  # unpack 
        
        # Origin of NED frame in Lat Lon
        origin_lat_rad = np.radians(63.9797)
        origin_lon_rad = np.radians(-22.5862)

        # Current NED pos
        ned_n_pos = data["X_x"]
        ned_e_pos = data["X_y"]
        ned_d_pos = data["X_z"]

        new_lat_rad, new_lon_rad, new_alt_m = ned_to_latlon(origin_lat_rad, origin_lon_rad, ned_n_pos, ned_e_pos, ned_d_pos)

        # Set FlightGear FDM fields
        fdm_data.alt_m = new_alt_m
        fdm_data.lon_rad = new_lon_rad
        fdm_data.lat_rad = new_lat_rad

        fdm_data.phi_rad = data["X_phi"]
        fdm_data.theta_rad = data["X_theta"]
        fdm_data.psi_rad = data["X_psi"]
        
        # Control surfaces
        fdm_data.elevator = data["u_de"]
        fdm_data.rudder = data["u_dr"]
        # Split aileron value between left and right
        fdm_data.left_aileron = -data["u_da"]
        fdm_data.right_aileron = data["u_da"]

    return fdm_data  # return the whole structure


def run_replay(csv_file):
    fdm_conn = FDMConnection()
    fdm_event_pipe = fdm_conn.connect_rx('127.0.0.1', 5501, fdm_callback)
    fdm_conn.connect_tx('127.0.0.1', 5502)
    fdm_conn.start()

    df = pd.read_csv(csv_file, skipinitialspace=True)
    
    df = df.replace('-nan(ind)', np.nan).astype(float)

    # use sim time from first row as reference
    start_sim_time = df["time"].iloc[0]
    start_real_time = time.time()

    for _, data in df.iterrows():
        sim_time = data["time"] - start_sim_time
        expected_real_time = start_real_time + sim_time

        # Wait until the correct wall-clock time
        while time.time() < expected_real_time:
            time.sleep(0.001)  # fine-grained wait

        fdm_event_pipe.parent_send((data,))


def run(csv_file: str = 'output/data_log.csv'):
    # 1. Launch FlightGear
    fg_proc = start_flightgear()

    # 2. Wait until FG is ready
    print("Waiting for FlightGear to fully initialize...")
    if wait_for_fg_ready():
        print("Adding 10s buffer...")
        time.sleep(10)
    else:
        print("Timeout waiting for FG readiness.")
        fg_proc.terminate()
        exit(1)

    # 3. Start replay
    run_replay(csv_file)


if __name__ == '__main__':
    run()