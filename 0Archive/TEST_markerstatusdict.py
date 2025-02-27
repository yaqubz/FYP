# 12 Feb OK - for understanding and clarity of marker_status for the server

marker_status = {
    "1": {"detected": True, "landed": False},   # is_marker_available("1") returns False
    "2": {"detected": False, "landed": True},   # is_marker_available("2") returns False
    "3": {"detected": False, "landed": False},  # is_marker_available("3") returns True
}

for marker_id in range(1,10):
    marker_data = marker_status.get(str(marker_id))  # Safely returns None if not found
    if marker_data:
        detected = marker_data.get("detected", False)  # Safely returns False if not found
        landed = marker_data.get("landed", False)      # Safely returns False if not found
        print(f"{marker_id}: {detected}, {landed}")




marker_status2 = {
    1: {"detected": True, "landed": False},   # is_marker_available("1") returns False
    2: {"detected": False, "landed": True},   # is_marker_available("2") returns False
    3: {"detected": False, "landed": False},  # is_marker_available("3") returns True
}

for marker_id in range(1,5):
    marker_data = marker_status2.get(marker_id)  # Safely returns None if not found
    if marker_data:
        detected = marker_data.get("detected", False)  # Safely returns False if not found
        landed = marker_data.get("landed", False)      # Safely returns False if not found
        print(f"{marker_id}: {detected}, {landed}")