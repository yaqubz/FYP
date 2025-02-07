import markerserverclient

marker_client = markerserverclient.MarkerClient()

print(marker_client.marker_status)
print(marker_client.is_marker_available(2))