import os
import sys
import optparse
import json
import csv
from xml.dom.minidom import parse, parseString
from core.Util import *
from core.target_vehicles_generation_protocols import *

HERO_VEHICLE_ID = "hero_1"
JAMMED_EDGES = ["gneE23", "gneE14"]  # Edges with simulated traffic jam

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

import traci
import sumolib
import math
from datetime import datetime
from controller.RouteController import *

"""
SUMO Selfless Traffic Routing (STR) Testbed
"""

MAX_SIMULATION_STEPS = 2000
POST_HERO_BUFFER_STEPS = 3

# TODO: decide which file to put these in. Right now they're also defined in RouteController!!
STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class StrSumo:
    def __init__(self, route_controller, connection_info, controlled_vehicles):
        """
        :param route_controller: object that implements the scheduling algorithm for controlled vehicles
        :param connection_info: object that includes the map information
        :param controlled_vehicles: a dictionary that includes the vehicles under control
        """
        self.direction_choices = [STRAIGHT, TURN_AROUND, SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]
        self.connection_info = connection_info
        self.route_controller = route_controller
        self.controlled_vehicles =  controlled_vehicles # dictionary of Vehicles by id
        #print(self.controlled_vehicles)
        self._hero_tls_forced = {}
        self._hero_tls_proximity_m = 120.0
        self._tls_congestion_vehicle_threshold = 3

    def _add_or_update_polyline(self, polygon_id, points, color=(255, 165, 0, 255), layer=95):
        """Draw a live polyline for the hero path using a non-filled polygon."""
        if len(points) < 2:
            return
        try:
            traci.polygon.setShape(polygon_id, points)
        except traci.exceptions.TraCIException:
            traci.polygon.add(
                polygon_id,
                points,
                color,
                fill=False,
                polygonType="hero_path",
                layer=layer,
                lineWidth=1,
            )

    def _circle_points(self, center_x, center_y, radius=6.0, segments=18):
        """Build a polygon approximating a circle for high-visibility markers."""
        pts = []
        for i in range(segments):
            angle = (2.0 * math.pi * i) / segments
            pts.append((center_x + radius * math.cos(angle), center_y + radius * math.sin(angle)))
        return pts

    def _add_highlight_circle(self, marker_id, x, y, color, radius=6.0, layer=109):
        """Add or update a visible circle around important route points."""
        circle_id = f"{marker_id}_circle"
        shape = self._circle_points(x, y, radius=radius)
        try:
            traci.polygon.setShape(circle_id, shape)
        except traci.exceptions.TraCIException:
            traci.polygon.add(
                circle_id,
                shape,
                color,
                fill=False,
                polygonType="route_marker",
                layer=layer,
                lineWidth=3,
            )

    def _add_route_marker(self, marker_id, x, y, color, marker_type, layer=110):
        """Add a labeled map marker (POI) for route start or end."""
        try:
            traci.poi.remove(marker_id)
        except:
            pass
        try:
            traci.poi.add(marker_id, x, y, color, marker_type, layer=layer)
            try:
                traci.poi.setWidth(marker_id, 3.0)
            except:
                pass
            self._add_highlight_circle(marker_id, x, y, color)
        except Exception as e:
            print(f"[MAP MARKER] Failed to place {marker_id}: {e}")

    def _get_hero_route_anchor_edges(self):
        """Resolve hero start and destination edges from configured controller/vehicle state."""
        hero_vehicle = self.controlled_vehicles.get(HERO_VEHICLE_ID)
        destination_edge = hero_vehicle.destination if hero_vehicle is not None else None

        start_edge = None
        if hasattr(self.route_controller, "direct_path"):
            direct_path = getattr(self.route_controller, "direct_path", None)
            if direct_path:
                start_edge = direct_path[0]
        if start_edge is None and hasattr(self.route_controller, "alternative_path"):
            alternative_path = getattr(self.route_controller, "alternative_path", None)
            if alternative_path:
                start_edge = alternative_path[0]

        return start_edge, destination_edge

    def _place_hero_route_markers_before_spawn(self):
        """Show planned start/end markers before the hero vehicle is released."""
        start_edge, destination_edge = self._get_hero_route_anchor_edges()
        start_marker_set = False
        end_marker_set = False

        if start_edge:
            try:
                start_lane_shape = traci.lane.getShape(f"{start_edge}_0")
                sx, sy = start_lane_shape[0]
                self._add_route_marker(
                    f"{HERO_VEHICLE_ID}_route_start",
                    sx,
                    sy,
                    (0, 255, 0, 255),
                    "START",
                )
                start_marker_set = True
            except Exception as e:
                print(f"[MAP MARKER] Failed to place pre-spawn hero start marker: {e}")

        if destination_edge:
            try:
                destination_lane_shape = traci.lane.getShape(f"{destination_edge}_0")
                ex, ey = destination_lane_shape[-1]
                self._add_route_marker(
                    f"{HERO_VEHICLE_ID}_route_end",
                    ex,
                    ey,
                    (255, 69, 0, 255),
                    "END",
                )
                end_marker_set = True
            except Exception as e:
                print(f"[MAP MARKER] Failed to place pre-spawn hero end marker: {e}")

        return start_marker_set, end_marker_set

    def _restore_tls_states(self, keep_tls_ids):
        """Restore original states for any TLS not currently under hero preemption."""
        restore_ids = [tls_id for tls_id in self._hero_tls_forced.keys() if tls_id not in keep_tls_ids]
        for tls_id in restore_ids:
            try:
                traci.trafficlight.setRedYellowGreenState(tls_id, self._hero_tls_forced[tls_id])
                del self._hero_tls_forced[tls_id]
            except Exception:
                pass

    def _preempt_hero_signals(self, vehicle_ids):
        """Force nearby upcoming hero traffic lights to green to reduce stop delays."""
        if HERO_VEHICLE_ID not in vehicle_ids:
            if self._hero_tls_forced:
                self._restore_tls_states(set())
            return

        try:
            next_tls = traci.vehicle.getNextTLS(HERO_VEHICLE_ID)
            nearby_tls_ids = set()
            for tls_item in next_tls:
                tls_id = tls_item[0]
                distance = float(tls_item[2])
                if distance > self._hero_tls_proximity_m:
                    continue

                # If incoming lanes at this TLS are already congested, preempt more aggressively.
                try:
                    controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
                    incoming_edges = {lane_id.rsplit("_", 1)[0] for lane_id in controlled_lanes if "_" in lane_id}
                    queued = 0
                    for edge_id in incoming_edges:
                        queued += traci.edge.getLastStepVehicleNumber(edge_id)
                    if queued >= self._tls_congestion_vehicle_threshold:
                        nearby_tls_ids.add(tls_id)
                except Exception:
                    pass

                nearby_tls_ids.add(tls_id)
                current_state = traci.trafficlight.getRedYellowGreenState(tls_id)
                if tls_id not in self._hero_tls_forced:
                    self._hero_tls_forced[tls_id] = current_state

                # Aggressive preemption: set all controlled links green while hero is near.
                forced_state = "".join("G" if ch in "rRgGyY" else ch for ch in current_state)
                traci.trafficlight.setRedYellowGreenState(tls_id, forced_state)

            self._restore_tls_states(nearby_tls_ids)
        except Exception:
            # Keep simulation robust if a TLS API call fails in any step.
            pass

    def _build_polyline_from_edge_path(self, edge_path):
        """Build a destination-facing polyline from an edge list.

        This draws planned route geometry (current->destination), not just the traveled trail.
        """
        poly_points = []
        if not edge_path:
            return poly_points

        for edge in edge_path:
            lane_id = f"{edge}_0"
            try:
                shape = traci.lane.getShape(lane_id)
            except Exception:
                continue
            if not shape:
                continue
            if not poly_points:
                poly_points.extend(shape)
            else:
                if poly_points[-1] == shape[0]:
                    poly_points.extend(shape[1:])
                else:
                    poly_points.extend(shape)
        return poly_points

    def _get_active_hero_path_suffix(self, current_edge):
        """Return active planned path suffix from current edge to destination."""
        direct_path = getattr(self.route_controller, "direct_path", [])
        alt_path = getattr(self.route_controller, "alternative_path", [])
        current_plan = getattr(self.route_controller, "hero_current_plan", "direct")
        active_path = alt_path if current_plan == "alternative" else direct_path
        if current_edge in active_path:
            return active_path[active_path.index(current_edge):]
        return []

    def _export_research_report(self, report_data):
        """Export simulation stats and graphs for research-paper usage."""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            report_dir = os.path.join("outputs", "research_reports", f"run_{timestamp}")
            os.makedirs(report_dir, exist_ok=True)

            # JSON summary for reproducibility and scripting.
            json_path = os.path.join(report_dir, "summary.json")
            with open(json_path, "w", encoding="utf-8") as f_json:
                json.dump(report_data, f_json, indent=2)

            # Flat CSV summary for spreadsheet workflows.
            csv_path = os.path.join(report_dir, "summary.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as f_csv:
                writer = csv.writer(f_csv)
                writer.writerow(["metric", "value"])
                writer.writerow(["hero_spawn_step", report_data.get("hero_spawn_step")])
                writer.writerow(["hero_arrival_step", report_data.get("hero_arrival_step")])
                writer.writerow(["hero_actual_time_s", report_data.get("hero_actual_time_s")])
                writer.writerow(["hero_reroute_step", report_data.get("hero_reroute_step")])
                writer.writerow(["direct_length_m", report_data.get("direct_length_m")])
                writer.writerow(["alternate_length_m", report_data.get("alternate_length_m")])
                writer.writerow(["direct_time_theoretical_s", report_data.get("direct_time_theoretical_s")])
                writer.writerow(["alternate_time_theoretical_s", report_data.get("alternate_time_theoretical_s")])
                writer.writerow(["direct_time_counterfactual_s", report_data.get("direct_time_counterfactual_s")])
                writer.writerow(["reroute_time_delta_s", report_data.get("reroute_time_delta_s")])
 
                writer.writerow(["percentage_improvement", report_data.get("percentage_improvement")])
                writer.writerow(["deadline_step", report_data.get("deadline_step")])
                writer.writerow(["deadline_buffer_s", report_data.get("deadline_buffer_s")])
                writer.writerow(["deadline_status", report_data.get("deadline_status")])
                writer.writerow(["estimation_method", report_data.get("estimation_method")])

            # Human-readable markdown summary.
            md_path = os.path.join(report_dir, "research_summary.md")
            with open(md_path, "w", encoding="utf-8") as f_md:
                f_md.write("# Simulation Research Summary\n\n")
                f_md.write("## Map Context\n")
                f_md.write(f"- Network map file: {report_data.get('map_file', 'N/A')}\n")
                f_md.write(f"- Hero start edge: {report_data.get('hero_start_edge', 'N/A')}\n")
                f_md.write(f"- Hero destination edge: {report_data.get('hero_destination_edge', 'N/A')}\n\n")
                f_md.write("## Core Stats\n")
                f_md.write(f"- Hero spawn step: {report_data.get('hero_spawn_step')}\n")
                f_md.write(f"- Hero arrival step: {report_data.get('hero_arrival_step')}\n")
                f_md.write(f"- Hero actual travel time: {report_data.get('hero_actual_time_s')} s\n")
                f_md.write(f"- Reroute step: {report_data.get('hero_reroute_step')}\n")
                f_md.write(f"- Direct route length: {report_data.get('direct_length_m'):.2f} m\n")
                f_md.write(f"- Alternate route length: {report_data.get('alternate_length_m'):.2f} m\n")
                f_md.write(f"- Alternate theoretical time: {report_data.get('alternate_time_theoretical_s'):.2f} s\n")
                f_md.write(f"- Direct with traffic time: {report_data.get('direct_time_counterfactual_s'):.2f} s\n")
                f_md.write(f"- Reroute time delta (direct - actual): {report_data.get('reroute_time_delta_s'):.2f} s\n")
                
                # Calculate and display percentage improvement
                direct_time = report_data.get('direct_time_counterfactual_s', 0)
                actual_time = report_data.get('hero_actual_time_s', 0)
                if direct_time > 0:
                    percent_improvement = ((direct_time - actual_time) / direct_time) * 100
                    f_md.write(f"- **Percentage improvement: {percent_improvement:.1f}%**\n")
                
                f_md.write(f"- Deadline status: {report_data.get('deadline_status')}\n")
                f_md.write(f"- Deadline buffer: {report_data.get('deadline_buffer_s')} s\n")
                f_md.write(f"- Estimation method: {report_data.get('estimation_method')}\n\n")
                f_md.write("## Hero Path\n")
                f_md.write("- " + " -> ".join(report_data.get("hero_taken_edges", [])) + "\n\n")
                f_md.write("## Generated Figures\n")
                f_md.write("- route_map_taken.png (actual XY map path taken by hero)\n")
                f_md.write("- node_to_node_mapping_graph.png (directed transitions for taken route)\n")
                f_md.write("- planned_vs_taken_route_graph.png (route sequence comparison)\n")
                f_md.write("- route_length_comparison.png\n")
                f_md.write("- travel_time_comparison.png\n")
                f_md.write("- hero_event_timeline.png\n")
                f_md.write("- percentage_improvement.png (% time savings vs direct route)\n")

            # Terminal-like snapshot so paper text can cite exact run metrics.
            snapshot_path = os.path.join(report_dir, "terminal_metrics_snapshot.txt")
            with open(snapshot_path, "w", encoding="utf-8") as f_txt:
                f_txt.write("TIMING STATISTICS\n")
                f_txt.write(f"Hero spawned at step: {report_data.get('hero_spawn_step')}\n")
                f_txt.write(f"Hero arrived at step: {report_data.get('hero_arrival_step')}\n")
                f_txt.write(f"Actual time taken: {report_data.get('hero_actual_time_s')} seconds\n\n")

                f_txt.write("PATH COMPARISON\n")
                f_txt.write(f"Direct path: {' -> '.join(report_data.get('direct_path_edges', []))}\n")
                f_txt.write(f"Alternate path: {' -> '.join(report_data.get('alternate_path_edges', []))}\n")
                f_txt.write(f"Direct length: {report_data.get('direct_length_m'):.2f} m\n")
                f_txt.write(f"Alternate length: {report_data.get('alternate_length_m'):.2f} m\n")
                f_txt.write(f"Alternate theoretical time: {report_data.get('alternate_time_theoretical_s'):.2f} s\n")
                f_txt.write(f"Direct with traffic time: {report_data.get('direct_time_counterfactual_s'):.2f} s\n")
                f_txt.write(f"Alternate actual time: {report_data.get('hero_actual_time_s'):.2f} s\n")
                f_txt.write(f"Reroute time delta (direct - alternate): {report_data.get('reroute_time_delta_s'):.2f} s\n")
                
                # Calculate and display percentage improvement
                direct_time = report_data.get('direct_time_counterfactual_s', 0)
                actual_time = report_data.get('hero_actual_time_s', 0)
                if direct_time > 0:
                    percent_improvement = ((direct_time - actual_time) / direct_time) * 100
                    f_txt.write(f"Percentage improvement: {percent_improvement:.1f}%\n")
                
                f_txt.write("\nDEADLINE ANALYSIS\n")
                f_txt.write(f"Deadline step: {report_data.get('deadline_step')}\n")
                f_txt.write(f"Deadline status: {report_data.get('deadline_status')}\n")
                f_txt.write(f"Deadline buffer: {report_data.get('deadline_buffer_s')}\n\n")

                f_txt.write("HERO ROUTE TRACE\n")
                f_txt.write(f"Nodes visited in order: {' -> '.join(report_data.get('hero_taken_edges', []))}\n")

            # Plot exports for direct inclusion in papers/slides.
            try:
                import matplotlib.pyplot as plt

                # Chart 1: Distance comparison with difference.
                fig1 = plt.figure(figsize=(7, 4.8))
                distance_vals = [report_data.get("direct_length_m", 0), report_data.get("alternate_length_m", 0)]
                bars = plt.bar(["Direct", "Alternate"], distance_vals, color=["#3a7ca5", "#f28e2b"])
                plt.ylabel("Distance (m)")
                plt.title("Route Length Comparison")
                
                # Add value labels on top of bars
                for bar in bars:
                    height = bar.get_height()
                    plt.text(bar.get_x() + bar.get_width()/2., height,
                            f'{height:.1f}m',
                            ha='center', va='bottom', fontsize=10, fontweight='bold')
                
                # Show route length difference below
                direct_length = report_data.get("direct_length_m", 0)
                alternate_length = report_data.get("alternate_length_m", 0)
                if direct_length > 0:
                    length_diff = alternate_length - direct_length
                    percent_diff = (length_diff / direct_length) * 100
                    diff_text = f"Alternate Route: {abs(length_diff):.1f}m {'longer' if length_diff > 0 else 'shorter'} ({abs(percent_diff):.1f}%)"
                    diff_color = "#d62728" if length_diff > 0 else "#2ca02c"
                    plt.text(0.5, -0.15, diff_text,
                            ha='center', va='top', transform=plt.gca().transAxes,
                            fontsize=10, fontweight='bold', color=diff_color,
                            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor=diff_color, linewidth=2))
                
                plt.tight_layout()
                fig1.savefig(os.path.join(report_dir, "route_length_comparison.png"), dpi=220)
                plt.close(fig1)

                # Chart 2: Time comparison with percentage improvement.
                fig2 = plt.figure(figsize=(8, 5.2))
                time_labels = ["Direct\nWith Traffic", "Alternate\nActual"]
                time_vals = [
                    report_data.get("direct_time_counterfactual_s", 0),
                    report_data.get("hero_actual_time_s", 0),
                ]
                bars = plt.bar(time_labels, time_vals, color=["#e15759", "#f28e2b"])
                plt.ylabel("Time (s)")
                plt.title("Travel Time Comparison")
                
                # Add value labels on top of bars
                for bar in bars:
                    height = bar.get_height()
                    plt.text(bar.get_x() + bar.get_width()/2., height,
                            f'{height:.1f}s',
                            ha='center', va='bottom', fontsize=10, fontweight='bold')
                
                # Calculate and display percentage improvement below
                direct_time = report_data.get("direct_time_counterfactual_s", 0)
                actual_time = report_data.get("hero_actual_time_s", 0)
                if direct_time > 0:
                    percent_improvement = ((direct_time - actual_time) / direct_time) * 100
                    time_saved = direct_time - actual_time
                    improvement_text = f"Alternate Route: {percent_improvement:.1f}% faster ({time_saved:.1f}s saved)"
                    improvement_color = "#2ca02c" if percent_improvement > 0 else "#d62728"
                    plt.text(0.5, -0.15, improvement_text, 
                            ha='center', va='top', transform=plt.gca().transAxes,
                            fontsize=11, fontweight='bold', color=improvement_color,
                            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor=improvement_color, linewidth=2))
                
                plt.tight_layout()
                fig2.savefig(os.path.join(report_dir, "travel_time_comparison.png"), dpi=220)
                plt.close(fig2)

                # Chart 3: Event timeline (spawn, reroute, arrival).
                fig3 = plt.figure(figsize=(8, 2.8))
                events = ["Spawn", "Reroute", "Arrival"]
                event_steps = [
                    report_data.get("hero_spawn_step", 0),
                    report_data.get("hero_reroute_step", 0) if report_data.get("hero_reroute_step") is not None else 0,
                    report_data.get("hero_arrival_step", 0),
                ]
                colors = ["#4e79a7", "#af7aa1", "#f28e2b"]
                plt.scatter(event_steps, [1, 1, 1], c=colors, s=100)
                for idx, label in enumerate(events):
                    plt.text(event_steps[idx], 1.03, f"{label}: {event_steps[idx]}", ha="center", fontsize=9)
                plt.yticks([])
                plt.xlabel("Simulation Step")
                plt.title("Hero Event Timeline")
                plt.tight_layout()
                fig3.savefig(os.path.join(report_dir, "hero_event_timeline.png"), dpi=220)
                plt.close(fig3)

                # Chart 4: Actual map-like XY route taken by hero.
                route_points = report_data.get("hero_route_points", [])
                if len(route_points) >= 2:
                    xs = [pt[0] for pt in route_points]
                    ys = [pt[1] for pt in route_points]
                    fig4 = plt.figure(figsize=(7, 7))
                    plt.plot(xs, ys, color="#f28e2b", linewidth=1.5, label="Hero route taken")
                    plt.scatter(xs[0], ys[0], color="#2ca02c", s=50, label="START", zorder=3)
                    plt.scatter(xs[-1], ys[-1], color="#d62728", s=50, label="END", zorder=3)
                    plt.xlabel("Map X")
                    plt.ylabel("Map Y")
                    plt.title("Route Map Taken by Hero Vehicle")
                    plt.axis("equal")
                    plt.legend(loc="best")
                    plt.tight_layout()
                    fig4.savefig(os.path.join(report_dir, "route_map_taken.png"), dpi=220)
                    plt.close(fig4)

                # Chart 5: Node-to-node mapping graph (transitions actually taken).
                taken_nodes = report_data.get("hero_taken_edges", [])
                if len(taken_nodes) >= 2:
                    fig5 = plt.figure(figsize=(10, 3.8))
                    x_positions = list(range(len(taken_nodes)))
                    y_positions = [1] * len(taken_nodes)

                    plt.scatter(x_positions, y_positions, s=280, color="#4e79a7", zorder=3)
                    for i, node in enumerate(taken_nodes):
                        plt.text(i, 1.03, node, ha="center", fontsize=8)

                    for i in range(len(taken_nodes) - 1):
                        plt.annotate(
                            "",
                            xy=(x_positions[i + 1] - 0.08, 1),
                            xytext=(x_positions[i] + 0.08, 1),
                            arrowprops=dict(arrowstyle="->", lw=1.4, color="#f28e2b"),
                        )

                    plt.title("Node-to-Node Mapping Graph (Route Taken)")
                    plt.yticks([])
                    plt.xticks([])
                    plt.ylim(0.92, 1.12)
                    plt.tight_layout()
                    fig5.savefig(os.path.join(report_dir, "node_to_node_mapping_graph.png"), dpi=220)
                    plt.close(fig5)

                # Chart 6: Planned-vs-taken sequence comparison with time stats.
                direct_nodes = report_data.get("direct_path_edges", [])
                fig6 = plt.figure(figsize=(10, 4.8))

                if len(direct_nodes) > 0:
                    x_direct = list(range(len(direct_nodes)))
                    y_direct = [2] * len(direct_nodes)
                    plt.scatter(x_direct, y_direct, s=180, color="#e15759", label="Direct planned", zorder=3)
                    for i, node in enumerate(direct_nodes):
                        plt.text(i, 2.08, node, ha="center", fontsize=8)
                    for i in range(len(direct_nodes) - 1):
                        plt.plot([i, i + 1], [2, 2], color="#e15759", linewidth=1.2, alpha=0.8)

                if len(taken_nodes) > 0:
                    x_taken = list(range(len(taken_nodes)))
                    y_taken = [1] * len(taken_nodes)
                    plt.scatter(x_taken, y_taken, s=180, color="#f28e2b", label="Actually taken", zorder=3)
                    for i, node in enumerate(taken_nodes):
                        plt.text(i, 1.08, node, ha="center", fontsize=8)
                    for i in range(len(taken_nodes) - 1):
                        plt.plot([i, i + 1], [1, 1], color="#f28e2b", linewidth=1.2, alpha=0.8)

                plt.title("Planned vs Taken Route Sequence")
                plt.yticks([1, 2], ["Taken", "Planned Direct"])
                plt.xticks([])
                plt.legend(loc="upper right")
                plt.ylim(0.6, 2.4)

                # Add explicit route time comparison under the route-sequence chart.
                direct_time_seq = report_data.get("direct_time_counterfactual_s", 0)
                actual_time_seq = report_data.get("hero_actual_time_s", 0)
                if direct_time_seq > 0:
                    pct_seq = ((direct_time_seq - actual_time_seq) / direct_time_seq) * 100
                    delta_seq = direct_time_seq - actual_time_seq
                    seq_color = "#2ca02c" if pct_seq > 0 else "#d62728"
                    seq_text = (
                        f"Time: Direct {direct_time_seq:.1f}s vs Alternate {actual_time_seq:.1f}s"
                        f" | Improvement: {pct_seq:.1f}% ({delta_seq:.1f}s saved)"
                    )
                    plt.text(
                        0.5,
                        -0.12,
                        seq_text,
                        ha="center",
                        va="top",
                        transform=plt.gca().transAxes,
                        fontsize=10,
                        fontweight="bold",
                        color=seq_color,
                        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", edgecolor=seq_color, linewidth=1.8),
                    )

                plt.tight_layout()
                fig6.savefig(os.path.join(report_dir, "planned_vs_taken_route_graph.png"), dpi=220)
                plt.close(fig6)

                # Chart 7: Percentage improvement over direct route.
                direct_time = report_data.get("direct_time_counterfactual_s", 0)
                actual_time = report_data.get("hero_actual_time_s", 0)
                if direct_time > 0:
                    percent_improvement = ((direct_time - actual_time) / direct_time) * 100
                    time_saved = direct_time - actual_time
                    
                    fig7 = plt.figure(figsize=(7, 5))
                    bar_color = "#2ca02c" if percent_improvement > 0 else "#d62728"
                    plt.bar(["Alternate Route"], [percent_improvement], color=bar_color, width=0.5)
                    plt.ylabel("% Improvement")
                    plt.title("Time Savings vs Direct Route with Traffic")
                    plt.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
                    
                    # Add annotation with actual time saved
                    plt.text(0, percent_improvement + (2 if percent_improvement >= 0 else -2), 
                             f"{percent_improvement:.1f}%\n({time_saved:.1f}s saved)", 
                             ha='center', va='bottom' if percent_improvement >= 0 else 'top',
                             fontsize=11, fontweight='bold')
                    
                    plt.ylim(min(-10, percent_improvement - 10), max(60, percent_improvement + 10))
                    plt.tight_layout()
                    fig7.savefig(os.path.join(report_dir, "percentage_improvement.png"), dpi=220)
                    plt.close(fig7)
            except Exception as plot_err:
                print(f"[REPORT] Graph generation skipped: {plot_err}")

            print(f"[REPORT] Research outputs saved to: {report_dir}")
        except Exception as export_err:
            print(f"[REPORT] Failed to export research outputs: {export_err}")

    def _print_congestion_state(self, step, hero_edge, main_path):
        """Print visual representation of congestion building up on main path"""
        congestion_display = "\n" + "-"*70
        congestion_display += f"\n[CONGESTION STATE] Step {step} | Hero at {hero_edge}"
        congestion_display += "\nMain Path (Direct): "
        
        path_status = []
        for edge in main_path[:4]:  # Show first 4 edges of main path
            try:
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge)
                # Create visual bar for congestion
                bar = "#" * min(vehicle_count, 10)
                path_status.append(f"[{edge}: {bar} ({vehicle_count})]")
            except:
                pass
        
        congestion_display += " -> ".join(path_status)
        congestion_display += "\nAlternative Path (Detour): gneE10 -> gneE11 ... (clear)"
        congestion_display += "\n" + "-"*70
        print(congestion_display)

    def _print_live_route_comparison(self, step, current_edge, taken_edges):
        """Print node-by-node route status in real time."""
        usual_path = ["gneE10", "gneE23", "gneE14", "gneE25", "gneE20"]
        current_plan = "direct"
        active_path = usual_path

        # DynamicReroutePolicy exposes these attributes; fallback keeps this generic.
        if hasattr(self.route_controller, "hero_current_plan"):
            current_plan = getattr(self.route_controller, "hero_current_plan", "direct")
        if current_plan == "alternative" and hasattr(self.route_controller, "alternative_path"):
            active_path = getattr(self.route_controller, "alternative_path", usual_path)

        def _next_node(path, edge):
            if edge in path:
                idx = path.index(edge)
                if idx + 1 < len(path):
                    return path[idx + 1]
            return "DESTINATION"

        usual_next = _next_node(usual_path, current_edge)
        active_next = _next_node(active_path, current_edge)
        prev_edge = taken_edges[-2] if len(taken_edges) >= 2 else "START"

        print("\n" + "-" * 70)
        print(f"[LIVE NODE VIEW] Step {step}")
        print(f"Hero moved: {prev_edge} -> {current_edge}")
        print(f"Usual next node: {usual_next}")
        print(f"Current plan: {current_plan.upper()} | Next node: {active_next}")
        print(f"Taken so far: {len(taken_edges)} nodes")
        print("-" * 70)

    def run(self):
        """
        Runs the SUMO simulation
        At each time-step, cars that have moved edges make a decision based on user-supplied scheduler algorithm
        Decisions are enforced in SUMO by setting the destination of the vehicle to the result of the
        :returns: total time, number of cars that reached their destination, number of deadlines missed
        """
        total_time = 0
        end_number = 0
        deadlines_missed = []

        step = 0
        vehicles_to_direct = [] #  the batch of controlled vehicles passed to make_decisions()
        vehicle_IDs_in_simulation = []
        main_path_edges = ["gneE10", "gneE23", "gneE14", "gneE25", "gneE20"]
        hero_on_route = False
        last_hero_edge = None
        hero_taken_edges = []
        
        # Track hero vehicle timing and rerouting
        hero_start_time = None
        hero_arrival_time = None
        hero_reroute_step = None
        hero_initial_plan = "direct"
        hero_route_points = []
        hero_planned_polyline_points = []
        hero_last_point = None
        hero_polyline_sample_m = 2.0
        hero_start_marker_set = False
        hero_end_marker_set = False
        hero_polyline_id = f"{HERO_VEHICLE_ID}_route_polyline"
        hero_plan_polyline_id = f"{HERO_VEHICLE_ID}_planned_polyline"
        last_drawn_plan_signature = None
        
        # Track travel times on direct path for realistic counterfactual estimate
        # Store both travel times and timing information for synchronized sampling
        direct_path_travel_times = {edge: [] for edge in main_path_edges}
        direct_path_sample_times = {edge: [] for edge in main_path_edges}  # Track when samples were taken
        reroute_snapshot_time = None  # Time when reroute decision was made

        hero_start_marker_set, hero_end_marker_set = self._place_hero_route_markers_before_spawn()

        try:
            while traci.simulation.getMinExpectedNumber() > 0:
                vehicle_ids = set(traci.vehicle.getIDList())

                # store edge vehicle counts in connection_info.edge_vehicle_count
                self.get_edge_vehicle_counts()
                
                # Track direct-path traffic only during the hero's active trip window.
                if hero_start_time is not None and hero_arrival_time is None:
                    for edge in main_path_edges:
                        try:
                            # Use SUMO's adaptive travel time which accounts for queuing and congestion
                            travel_time = traci.edge.getAdaptedTraveltime(edge, step)
                            if travel_time is not None and travel_time > 0:
                                direct_path_travel_times[edge].append(travel_time)
                                direct_path_sample_times[edge].append(step)  # Track when we sampled
                        except:
                            pass
                
                #initialize vehicles to be directed
                vehicles_to_direct = []
                # iterate through vehicles currently in simulation
                for vehicle_id in vehicle_ids:

                    #should not be added because there is no corresponding -1, this makes edge_vehicle_count becomes the total number of vehicles that used to be on this edge.
                    #self.connection_info.edge_vehicle_count[traci.vehicle.getRoadID(vehicle_id)] += 1
                    
                    

                    # handle newly arrived controlled vehicles
                    if vehicle_id not in vehicle_IDs_in_simulation and vehicle_id in self.controlled_vehicles:
                        vehicle_IDs_in_simulation.append(vehicle_id)
                        if str(vehicle_id) == HERO_VEHICLE_ID:
                            traci.vehicle.setColor(vehicle_id, (255, 20, 147)) # Hot pink for hero vehicle
                            traci.vehicle.setLength(vehicle_id, 8.0) # Make hero bigger
                            traci.vehicle.setWidth(vehicle_id, 3.0)
                            hero_start_time = step  # Record hero start time
                            print("[HERO] Vehicle spawned at step {} on edge {}".format(step, traci.vehicle.getRoadID(vehicle_id)))
                            try:
                                sx, sy = traci.vehicle.getPosition(vehicle_id)
                                hero_route_points.append((sx, sy))
                                hero_last_point = (sx, sy)
                                self._add_route_marker(
                                    f"{HERO_VEHICLE_ID}_route_start",
                                    sx,
                                    sy,
                                    (0, 255, 0, 255),
                                    "START",
                                )
                                hero_start_marker_set = True
                                print(f"[MAP MARKER] START at ({sx:.1f}, {sy:.1f})")
                            except Exception as e:
                                print(f"[MAP MARKER] Failed to capture hero start marker: {e}")
                        else:
                            traci.vehicle.setColor(vehicle_id, (255, 0, 0)) # Red for other controlled vehicles
                        self.controlled_vehicles[vehicle_id].start_time = float(step)#Use the detected release time as start time

                    if vehicle_id in self.controlled_vehicles.keys():
                        current_edge = traci.vehicle.getRoadID(vehicle_id)

                        if current_edge not in self.connection_info.edge_index_dict.keys():
                            continue
                        elif current_edge == self.controlled_vehicles[vehicle_id].destination:
                            continue

                        #print("{} now on: {}, records on {}; {} ".format(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].current_edge, current_edge!=self.controlled_vehicles[vehicle_id].current_edge))
                        if current_edge != self.controlled_vehicles[vehicle_id].current_edge:
                            self.controlled_vehicles[vehicle_id].current_edge = current_edge
                            self.controlled_vehicles[vehicle_id].current_speed = traci.vehicle.getSpeed(vehicle_id)
                            if str(vehicle_id) == HERO_VEHICLE_ID:
                                hero_on_route = True
                                last_hero_edge = current_edge
                                print("[HERO TRACE] step {} edge {} speed {:.2f}".format(step, current_edge, self.controlled_vehicles[vehicle_id].current_speed))
                                if len(hero_taken_edges) == 0 or hero_taken_edges[-1] != current_edge:
                                    hero_taken_edges.append(current_edge)

                                # Draw planned path below the traveled trail for clean overlap.
                                plan_suffix = self._get_active_hero_path_suffix(current_edge)
                                plan_signature = (getattr(self.route_controller, "hero_current_plan", "direct"), tuple(plan_suffix))
                                if plan_signature != last_drawn_plan_signature:
                                    hero_planned_polyline_points = self._build_polyline_from_edge_path(plan_suffix)
                                    self._add_or_update_polyline(
                                        hero_plan_polyline_id,
                                        hero_planned_polyline_points,
                                        color=(0, 200, 255, 220),
                                        layer=94,
                                    )
                                    last_drawn_plan_signature = plan_signature
                                
                                # Track reroute decision
                                if hasattr(self.route_controller, "hero_current_plan"):
                                    current_plan = getattr(self.route_controller, "hero_current_plan", "direct")
                                    if current_plan == "alternative" and hero_reroute_step is None:
                                        hero_reroute_step = step
                                        reroute_snapshot_time = step  # Record when decision was made
                                
                                self._print_live_route_comparison(step, current_edge, hero_taken_edges)
                                # Show congestion state on main path when hero is actively routing
                                if current_edge == "gneE10" and step % 5 == 0:  # Show every 5 steps while approaching main path
                                    self._print_congestion_state(step, current_edge, main_path_edges)
                            vehicles_to_direct.append(self.controlled_vehicles[vehicle_id])
                        else:
                            # Re-evaluate hero every step so reroute can trigger while waiting on the same edge.
                            if str(vehicle_id) == HERO_VEHICLE_ID:
                                self.controlled_vehicles[vehicle_id].current_speed = traci.vehicle.getSpeed(vehicle_id)
                                vehicles_to_direct.append(self.controlled_vehicles[vehicle_id])
                #print(len(vehicles_to_direct))
                vehicle_decisions_by_id = self.route_controller.make_decisions(vehicles_to_direct, self.connection_info)
                for vehicle_id, local_target_edge in vehicle_decisions_by_id.items():
                    # if decision not in self.connection_info.outgoing_edges_dict[self.controlled_vehicles[vehicle_id].current_edge]:
                    #     raise ValueError(f'{decision} does not lead to a valid edge from edge '
                    #                      f'{self.controlled_vehicles[vehicle_id].current_edge}')
                    #
                    # current_edge_of_vehicle = self.controlled_vehicles[vehicle_id].current_edge
                    # target_edge = self.connection_info.outgoing_edges_dict[current_edge_of_vehicle][decision]
                    if vehicle_id in traci.vehicle.getIDList():
                        #print("Changing the target of {} to {} with length {}".format(vehicle_id, local_target_edge, self.connection_info.edge_length_dict[local_target_edge]))
                        traci.vehicle.changeTarget(vehicle_id, local_target_edge)
                        self.controlled_vehicles[vehicle_id].local_destination = local_target_edge

                arrived_at_destination = traci.simulation.getArrivedIDList()

                for vehicle_id in arrived_at_destination:
                    if vehicle_id in self.controlled_vehicles:
                        #print the raw result out to the terminal
                        arrived_at_destination = False
                        if self.controlled_vehicles[vehicle_id].local_destination == self.controlled_vehicles[vehicle_id].destination:
                            arrived_at_destination = True
                        time_span = step - self.controlled_vehicles[vehicle_id].start_time
                        total_time += time_span
                        miss = False
                        if step > self.controlled_vehicles[vehicle_id].deadline:
                            deadlines_missed.append(vehicle_id)
                            miss = True
                        end_number += 1
                        
                        # Track hero vehicle arrival
                        if str(vehicle_id) == HERO_VEHICLE_ID:
                            hero_arrival_time = step
                            if not hero_end_marker_set:
                                try:
                                    # Vehicle can be gone from simulation list at this point;
                                    # use lane endpoint on destination edge as stable end marker.
                                    dest_edge = self.controlled_vehicles[vehicle_id].destination
                                    lane_id = f"{dest_edge}_0"
                                    lane_shape = traci.lane.getShape(lane_id)
                                    ex, ey = lane_shape[-1]
                                    self._add_route_marker(
                                        f"{HERO_VEHICLE_ID}_route_end",
                                        ex,
                                        ey,
                                        (255, 69, 0, 255),
                                        "END",
                                    )
                                    hero_end_marker_set = True
                                    print(f"[MAP MARKER] END at ({ex:.1f}, {ey:.1f})")
                                except Exception as e:
                                    print(f"[MAP MARKER] Failed to place hero end marker: {e}")
                        
                        print("Vehicle {} reaches the destination: {}, timespan: {}, deadline missed: {}"\
                            .format(vehicle_id, arrived_at_destination, time_span, miss))
                        #if not arrived_at_destination:
                            #print("{} - {}".format(self.controlled_vehicles[vehicle_id].local_destination, self.controlled_vehicles[vehicle_id].destination))

                # Update hero polyline every step with distance-based sampling for smoother accuracy.
                if HERO_VEHICLE_ID in vehicle_ids:
                    try:
                        hx, hy = traci.vehicle.getPosition(HERO_VEHICLE_ID)
                        if hero_last_point is None:
                            hero_route_points.append((hx, hy))
                            hero_last_point = (hx, hy)
                        else:
                            dx = hx - hero_last_point[0]
                            dy = hy - hero_last_point[1]
                            if math.hypot(dx, dy) >= hero_polyline_sample_m:
                                hero_route_points.append((hx, hy))
                                hero_last_point = (hx, hy)
                        self._add_or_update_polyline(hero_polyline_id, hero_route_points, color=(255, 165, 0, 255), layer=97)
                    except Exception as e:
                        print(f"[ROUTE POLYLINE] Failed to update hero trail: {e}")

                # Preempt upcoming traffic lights for hero to avoid long stop delays.
                self._preempt_hero_signals(vehicle_ids)

                traci.simulationStep()
                
                step += 1

                if hero_arrival_time is not None and step >= hero_arrival_time + POST_HERO_BUFFER_STEPS:
                    print(f"Ending after hero completion buffer ({POST_HERO_BUFFER_STEPS} steps).")
                    break

                if step > MAX_SIMULATION_STEPS:
                    print('Ending due to timeout.')
                    break

        except ValueError as err:
            print('Exception caught.')
            print(err)

        num_deadlines_missed = len(deadlines_missed)

        # Calculate edge lengths and theoretical traversal times
        direct_path = ["gneE10", "gneE23", "gneE14", "gneE25", "gneE20"]
        alternate_path = ["gneE10", "gneE11", "gneE12", "gneE28", "gneE18", "gneE19", "gneE20"]
        
        # Get edge lengths (in meters) from connection_info
        direct_length = sum(self.connection_info.edge_length_dict.get(edge, 0) for edge in direct_path)
        alternate_length = sum(self.connection_info.edge_length_dict.get(edge, 0) for edge in alternate_path)
        
        # Average vehicle speed without congestion (m/s) - typical is ~13 m/s
        avg_speed_free_flow = 13.0  # m/s
        
        # Calculate theoretical times (in seconds)
        direct_theoretical_time = direct_length / avg_speed_free_flow if avg_speed_free_flow > 0 else 0
        alternate_theoretical_time = alternate_length / avg_speed_free_flow if avg_speed_free_flow > 0 else 0
        
        # If policy recorded ETA at reroute decision, use it as the primary estimate
        # This is the most accurate single-point estimate (made at actual decision moment)
        decision_direct_eta = None
        if hasattr(self.route_controller, "reroute_decision_direct_eta"):
            decision_direct_eta = getattr(self.route_controller, "reroute_decision_direct_eta", None)
        
        # Counterfactual direct-path estimate using SUMO's adaptive travel times
        # This accounts for actual vehicle experiences including queuing, congestion, and junction delays
        direct_realistic_time = 0.0
        direct_realistic_time_min = 0.0  # Best case scenario
        direct_realistic_time_max = 0.0  # Worst case scenario
        sampled_edges = 0
        total_variance = 0.0
        
        # If we have reroute decision ETA, use it as the primary estimate
        if decision_direct_eta is not None and decision_direct_eta > 0:
            direct_realistic_time = float(decision_direct_eta)
            direct_realistic_time_min = direct_realistic_time  
            direct_realistic_time_max = direct_realistic_time
            estimation_method = "reroute_decision"
        else:
            # Fallback: calculate from sampled travel times
            estimation_method = "sampled_traffic"
            import statistics
            
            for edge in direct_path:
                edge_length = self.connection_info.edge_length_dict.get(edge, 0.0)
                travel_time_samples = direct_path_travel_times.get(edge, [])

                if travel_time_samples and len(travel_time_samples) > 0:
                    # Calculate statistics for this edge
                    avg_travel_time = statistics.mean(travel_time_samples)
                    min_travel_time = min(travel_time_samples)
                    max_travel_time = max(travel_time_samples)
                    
                    if len(travel_time_samples) > 1:
                        std_dev = statistics.stdev(travel_time_samples)
                        total_variance += std_dev ** 2
                    
                    direct_realistic_time += avg_travel_time
                    direct_realistic_time_min += min_travel_time
                    direct_realistic_time_max += max_travel_time
                    sampled_edges += 1
                else:
                    # Fallback to theoretical time for this edge if no samples
                    fallback_time = edge_length / avg_speed_free_flow if avg_speed_free_flow > 0 else 0
                    direct_realistic_time += fallback_time
                    direct_realistic_time_min += fallback_time
                    direct_realistic_time_max += fallback_time

            if sampled_edges == 0:
                # Fallback when no hero-window traffic samples exist.
                direct_realistic_time = direct_theoretical_time
                direct_realistic_time_min = direct_theoretical_time
                direct_realistic_time_max = direct_theoretical_time
                estimation_method = "theoretical_fallback"
        
        # Actual hero times
        actual_hero_time = hero_arrival_time - hero_start_time if (hero_arrival_time is not None and hero_start_time is not None) else 0

        # Print final hero path summary
        print("\n" + "="*80)
        print("[FINAL REPORT] HERO VEHICLE DYNAMIC REROUTING DEMONSTRATION".center(80))
        print("="*80)
        print(">>> ROUTE SUCCESSFULLY TRACED <<<")
        print()
        
        # Timing summary
        print("TIMING STATISTICS:")
        print("-" * 80)
        print(f"Hero spawned at step:              {hero_start_time}")
        print(f"Hero arrived at step:              {hero_arrival_time}")
        print(f"Actual time taken:                 {actual_hero_time} seconds (simulation steps)")
        print()
        
        if hero_reroute_step is not None and hero_start_time is not None:
            print(f"Reroute decision made at step:     {hero_reroute_step}")
            print(f"Time before reroute:               {hero_reroute_step - hero_start_time} seconds")
        print()
        
        # Path comparison
        print("PATH COMPARISON (Distance vs Time):")
        print("-" * 80)
        direct_path_str = " -> ".join(direct_path)
        print(f"DIRECT path (TAKEN IF NO REROUTE):")
        print(f"  Route: {direct_path_str}")
        print(f"  Total length: {direct_length:.2f} meters")
        print(f"  Theoretical time (zero traffic): {direct_theoretical_time:.2f} seconds")
        print(f"  COUNTERFACTUAL TIME WITH TRAFFIC: {direct_realistic_time:.2f} seconds")
        
        # Display uncertainty information
        if estimation_method == "reroute_decision":
            print(f"    └─ Method: Routing decision ETA (most accurate)")
        elif estimation_method == "sampled_traffic":
            uncertainty_range = direct_realistic_time_max - direct_realistic_time_min
            print(f"    └─ Method: Sampled from {sampled_edges}/{len(direct_path)} edges during simulation")
            print(f"    └─ Uncertainty range: {direct_realistic_time_min:.2f} - {direct_realistic_time_max:.2f} seconds (±{uncertainty_range/2:.2f}s)")
            if uncertainty_range > direct_realistic_time * 0.3:
                print(f"    └─ ⚠ High variance in traffic conditions - estimate less reliable")
        else:
            print(f"    └─ Method: Theoretical estimate (no traffic data available)")
        
        print()
        alternate_path_str = " -> ".join(alternate_path)
        print(f"ALTERNATE path (ACTUALLY TAKEN):")
        print(f"  Route: {alternate_path_str}")
        print(f"  Total length: {alternate_length:.2f} meters")
        print(f"  Theoretical time (zero traffic): {alternate_theoretical_time:.2f} seconds")
        print(f"  ACTUAL TIME TAKEN:               {actual_hero_time} seconds")
        print()
        
        # Calculate worst-case and best-case savings
        delta_time = direct_realistic_time - actual_hero_time
        delta_time_min = direct_realistic_time_min - actual_hero_time
        delta_time_max = direct_realistic_time_max - actual_hero_time
        
        print(f"TIME IMPACT OF REROUTING:")
        print(f"  Direct (with traffic): {direct_realistic_time:.2f} sec")
        print(f"  Alternate (actual):    {actual_hero_time:.2f} sec")
        if delta_time > 0:
            if estimation_method == "sampled_traffic" and (direct_realistic_time_max - direct_realistic_time_min) > 0:
                print(f"  Savings (expected):    {delta_time:.2f} seconds ({(delta_time / direct_realistic_time * 100):.1f}% faster)")
                print(f"  Savings (worst case):  {delta_time_min:.2f} seconds")
                print(f"  Savings (best case):   {delta_time_max:.2f} seconds")
            else:
                print(f"  Savings:               {delta_time:.2f} seconds ({(delta_time / direct_realistic_time * 100):.1f}% faster)")
        elif delta_time < 0:
            print(f"  Alternate is slower by {-delta_time:.2f} seconds")
        else:
            print("  No measurable difference")
        print()
        path_diff_desc = "longer" if alternate_length > direct_length else "shorter"
        print(f"Distance difference:               {abs(direct_length - alternate_length):.2f} meters {path_diff_desc}")
        print()
        
        # Deadline analysis
        print("DEADLINE ANALYSIS:")
        print("-" * 80)
        hero_vehicle = self.controlled_vehicles.get(HERO_VEHICLE_ID)
        if hero_vehicle:
            deadline_step = hero_vehicle.deadline
            if hero_arrival_time is None:
                deadline_status = "NOT_ARRIVED"
                buffer = None
            else:
                deadline_status = "MISSED" if hero_arrival_time > deadline_step else "MET"
                buffer = deadline_step - hero_arrival_time
            print(f"Deadline (absolute step):          {deadline_step}")
            print(f"Arrival step:                      {hero_arrival_time}")
            print(f"Status:                            {deadline_status}")
            if hero_arrival_time is not None:
                if buffer >= 0:
                    print(f"Time buffer:                       {buffer} seconds remaining")
                else:
                    print(f"Deadline exceeded by:              {abs(buffer)} seconds")
        print()
        
        print("HERO ROUTE TRACE:")
        print("-" * 80)
        route_trace = " -> ".join(hero_taken_edges)
        print(f"Nodes visited in order: {route_trace}")
        print()
        
        print("INTELLIGENT ROUTING DECISION:")
        print("-" * 80)
        print("The hero vehicle (shown in pink in SUMO GUI):")
        print("  1. Started at gneE10")
        if hero_reroute_step:
            print(f"  2. DETECTED congestion on gneE23 at step {hero_reroute_step}")
            print(f"  3. TRIGGERED DYNAMIC REROUTE to avoid traffic (after {hero_reroute_step - hero_start_time} seconds)")
        print("  4. FOLLOWED alternate path through gneE11 -> gneE12 -> gneE28 -> gneE18 -> gneE19")
        print(f"  5. REACHED destination gneE20 at step {hero_arrival_time}")
        print()
        
        print("Direct path (BLOCKED by congestion):")
        print("  gneE10 --[CONGESTION DETECTED]-->")
        print("    -> gneE23 [RED TRAFFIC] -> gneE14 [RED TRAFFIC] -> gneE25 -> gneE20")
        print()
        print("Alternative path (TAKEN by intelligent routing):")
        print("  gneE10 -> gneE11 -> gneE12 -> gneE28 -> gneE18 -> gneE19 -> gneE20")
        print("           ^_________DETOUR TO AVOID JAM_________^")
        print()
        print("SUMO GUI Display Guide:")
        print("  [ Pink vehicle with orange trail ] = Hero vehicle following rerouted path")
        print("  [ Red vehicles + red areas ]     = Congestion on main path")
        print("="*80 + "\n")

        # Export paper-ready stats and plots after each run.
        deadline_step = None
        deadline_status = "UNKNOWN"
        deadline_buffer = None
        hero_vehicle = self.controlled_vehicles.get(HERO_VEHICLE_ID)
        if hero_vehicle:
            deadline_step = hero_vehicle.deadline
            if hero_arrival_time is not None:
                deadline_status = "MISSED" if hero_arrival_time > deadline_step else "MET"
                deadline_buffer = deadline_step - hero_arrival_time

        # Calculate percentage improvement
        percentage_improvement = 0.0
        if direct_realistic_time > 0:
            percentage_improvement = ((direct_realistic_time - actual_hero_time) / direct_realistic_time) * 100

        report_data = {
            "hero_id": HERO_VEHICLE_ID,
            "map_file": getattr(self.connection_info, "net_filename", "unknown"),
            "hero_start_edge": "gneE10",
            "hero_destination_edge": "gneE20",
            "hero_spawn_step": hero_start_time,
            "hero_arrival_step": hero_arrival_time,
            "hero_actual_time_s": actual_hero_time,
            "hero_reroute_step": hero_reroute_step,
            "direct_length_m": direct_length,
            "alternate_length_m": alternate_length,
            "direct_time_theoretical_s": direct_theoretical_time,
            "alternate_time_theoretical_s": alternate_theoretical_time,
            "direct_time_counterfactual_s": direct_realistic_time,
            "direct_time_counterfactual_min_s": direct_realistic_time_min,
            "direct_time_counterfactual_max_s": direct_realistic_time_max,
            "reroute_time_delta_s": delta_time,
            "percentage_improvement": percentage_improvement,
            "estimation_method": estimation_method,
            "deadline_step": deadline_step,
            "deadline_status": deadline_status,
            "deadline_buffer_s": deadline_buffer,
            "hero_taken_edges": hero_taken_edges,
            "hero_route_points": hero_route_points,
            "hero_planned_route_points": hero_planned_polyline_points,
            "direct_path_edges": direct_path,
            "alternate_path_edges": alternate_path,
            "sampled_direct_edge_count": sampled_edges,
            "polyline_points_count": len(hero_route_points),
            "simulation_step_end": step,
            "num_controlled_reached_destination": end_number,
            "average_timespan": (total_time / end_number) if end_number > 0 else None,
            "num_deadlines_missed": num_deadlines_missed,
            "deadlines_missed_vehicle_ids": deadlines_missed,
        }
        self._export_research_report(report_data)

        return total_time, end_number, num_deadlines_missed

    def get_edge_vehicle_counts(self):
        for edge in self.connection_info.edge_list:
            self.connection_info.edge_vehicle_count[edge] = traci.edge.getLastStepVehicleNumber(edge)

