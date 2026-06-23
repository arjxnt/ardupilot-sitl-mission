"""Render a flight log (from MissionResult.flight_log) to a PNG for the README/demo."""


def plot_flight_log(flight_log, target_lat, target_lon, target_radius_m, out_path):
    import matplotlib.pyplot as plt

    search_pts = [(lon, lat) for lat, lon, alt, label in flight_log if label == "search"]
    orbit_pts = [(lon, lat) for lat, lon, alt, label in flight_log if label == "orbit"]

    fig, ax = plt.subplots(figsize=(7, 7))
    if search_pts:
        xs, ys = zip(*search_pts)
        ax.plot(xs, ys, color="#2563eb", linewidth=1.5, label="Search pattern")
    if orbit_pts:
        xs, ys = zip(*orbit_pts)
        ax.plot(xs, ys, color="#dc2626", linewidth=1.5, label="Orbit")

    target_circle = plt.Circle(
        (target_lon, target_lat), target_radius_m / 111_320.0,
        color="#16a34a", fill=False, linestyle="--", label="Target geofence",
    )
    ax.add_patch(target_circle)
    ax.scatter([target_lon], [target_lat], color="#16a34a", marker="x", s=80)

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Search & Orbit Flight Path")
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
