# BAQIR 9 MAR OKAY

import socket
import time
import pandas as pd
import numpy as np

# Set the local IP of the D-Link adapter (connected to the UWB console’s router)
LOCAL_IP = "192.168.0.100"  # Replace with the actual IP of your DWA-72 adapter
UDP_PORT = 5000

PERIOD_S = 0.1

def parse_data_to_df(data):
    """
    Parse received UDP data into a pandas DataFrame.
    """
    try:
        # Split the received data into lines
        lines = data.decode().splitlines()
        rows = []
        for line in lines:
            values = line.split(',')
            if len(values) == 13:  # Ensure we have all expected values
                row = {
                    'id': int(values[0]),
                    'role': int(values[1]),
                    'x': float(values[2]),
                    'y': float(values[3]),
                    'z': float(values[4]),
                    'dist1': float(values[5]),
                    'dist2': float(values[6]),
                    'dist3': float(values[7]),
                    'dist4': float(values[8]),
                    'dist5': float(values[9]),
                    'dist6': float(values[10]),
                    'dist7': float(values[11]),
                    'dist8': float(values[12])
                }
                rows.append(row)
        df = pd.DataFrame(rows)
        return df
    except Exception as e:
        print(f"Error parsing data to DataFrame: {e}")
        return None

def get_target_position(target_id, max_retries=3, timeout=0.1):
    """
    Get the position of a specific target ID via UDP communication.
    Returns (x,y,z) coordinates or (0,0,0) on failure.
    """
    GOT_POS = False
    retry_count = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind explicitly to the IP of the D-Link adapter
    server_address = (LOCAL_IP, UDP_PORT)
    sock.bind(server_address)
    sock.settimeout(timeout)

    while not GOT_POS and retry_count < max_retries:
        try:
            data, address = sock.recvfrom(4096)
            lines = data.decode().splitlines()
            for line in lines:
                parts = line.split(',')
                node_id = int(parts[0])
                if node_id == target_id:
                    GOT_POS = True
                    pos = (float(parts[2]), float(parts[3]), float(parts[4]))
                    print(f"Target {target_id}: {pos}")
                    sock.close()
                    return pos
            retry_count += 1
        except socket.timeout:
            print(f"[WARNING] Timeout occurred. Retry {retry_count + 1} of {max_retries}.")
            retry_count += 1
        except Exception as e:
            print(f"Error parsing data: {e}")
            retry_count += 1

    print(f"[WARNING] Failed to get position for ID{target_id}. Returning (0, 0, 0).")
    sock.close()
    return (0, 0, 0)

def get_all_positions(max_retries=3, timeout=0.2):
    """
    Get positions of all tags from one UDP receive operation.
    Returns a DataFrame with tag positions and distances.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind to the D-Link adapter IP
    server_address = (LOCAL_IP, UDP_PORT)
    sock.bind(server_address)
    sock.settimeout(timeout)

    retry_count = 0
    while retry_count < max_retries:
        try:
            data, address = sock.recvfrom(4096)
            df = parse_data_to_df(data)
            if df is not None and not df.empty:
                sock.close()
                return df
        except socket.timeout:
            retry_count += 1
        except Exception as e:
            print(f"Error receiving/processing data: {e}")
            retry_count += 1

    print(f"[WARNING] Failed to get positions after {max_retries} retries.")
    sock.close()
    return pd.DataFrame()

# Main routine for testing the functions
if __name__ == "__main__":
    while True:
        pos0 = get_target_position(target_id=0, max_retries=5)
        pos1 = get_target_position(target_id=1, max_retries=5)
        time.sleep(PERIOD_S)

        df = get_all_positions()
        if not df.empty:
            print("\nCurrent positions:")
            print(df[['id', 'x', 'y', 'z']].to_string(index=False))
        time.sleep(PERIOD_S)
