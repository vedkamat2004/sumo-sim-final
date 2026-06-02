from controller.AStarController import AStarPolicy
import traci


class DynamicReroutePolicy(AStarPolicy):
    """
    Policy that dynamically reroutes hero vehicle when detecting traffic congestion.
    Monitors edge congestion and switches between direct and alternative paths.
    """

    def __init__(self, connection_info, hero_vehicle_id, direct_path, alternative_path, congestion_threshold=5):
        super().__init__(connection_info)
        self.hero_vehicle_id = str(hero_vehicle_id)
        self.direct_path = list(direct_path)
        self.alternative_path = list(alternative_path)
        self.congestion_threshold = congestion_threshold
        self.hero_current_plan = "direct"  # Start with direct path
        self._last_switch_time = -1.0
        self._switch_cooldown_s = 2.0
        self.reroute_decision_direct_eta = None
        self.reroute_decision_alternative_eta = None

    def _choose_congestion_aware_target(self, vehicle, destination_edge):
        """Pick the best immediate outgoing edge when path-suffix routing is not usable.

        This keeps rerouting dynamic even when the vehicle is already stuck on an edge
        that is not part of the currently preferred path suffix.
        """
        current_edge = vehicle.current_edge
        outgoing = self.connection_info.outgoing_edges_dict.get(current_edge, {})
        if not outgoing:
            return None

        now = float(traci.simulation.getTime())
        best_edge = None
        best_score = float("inf")

        for out_edge in outgoing.values():
            try:
                travel_t = float(traci.edge.getAdaptedTraveltime(out_edge, now))
                if travel_t <= 0:
                    raise ValueError("invalid travel time")
            except Exception:
                edge_len = float(self.connection_info.edge_length_dict.get(out_edge, 40.0))
                speed = max(0.3, float(traci.edge.getLastStepMeanSpeed(out_edge)))
                travel_t = edge_len / speed

            try:
                occ_raw = float(traci.edge.getLastStepOccupancy(out_edge))
                occ_ratio = (occ_raw / 100.0) if occ_raw > 1.0 else occ_raw
            except Exception:
                occ_ratio = 0.0

            try:
                veh_n = float(traci.edge.getLastStepVehicleNumber(out_edge))
                edge_len = float(self.connection_info.edge_length_dict.get(out_edge, 40.0))
                density = veh_n / max(1.0, edge_len / 100.0)
            except Exception:
                density = 0.0

            heuristic = self._heuristic(out_edge, destination_edge)
            score = travel_t + (heuristic / 13.0) + (occ_ratio * 40.0) + (density * 6.0)
            if score < best_score:
                best_score = score
                best_edge = out_edge

        return best_edge

    def _check_route_congestion(self, current_edge, planned_path):
        """
        Checks if upcoming edges on the planned route are congested.
        Returns True if congestion detected.
        """
        if current_edge not in planned_path:
            return False
        
        current_index = planned_path.index(current_edge)
        # Check ALL edges ahead for congestion (not just next few)
        edges_to_check = planned_path[current_index + 1:]  # Start from NEXT edge, not current
        
        for edge in edges_to_check:
            try:
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge)
                edge_length = self.connection_info.edge_length_dict.get(edge, 100)
                density = vehicle_count / (edge_length / 100)  # vehicles per 100m
                
                if density > self.congestion_threshold:
                    print(f"[CONGESTION DETECTED] Edge {edge}: {vehicle_count} vehicles, density={density:.4f}")
                    return True
            except Exception as e:
                pass
        
        return False

    def _edge_path_to_decisions(self, current_edge, path):
        """
        Converts a suffix of the path into SUMO direction decisions.
        """
        if current_edge not in path:
            return []

        current_index = path.index(current_edge)
        decisions = []

        for i in range(current_index, len(path) - 1):
            from_edge = path[i]
            to_edge = path[i + 1]
            edge_options = self.connection_info.outgoing_edges_dict.get(from_edge, {})

            matched_direction = None
            for direction, outgoing_edge in edge_options.items():
                if outgoing_edge == to_edge:
                    matched_direction = direction
                    break

            if matched_direction is None:
                return []

            decisions.append(matched_direction)

        return decisions

    def _estimate_path_time(self, current_edge, path):
        """
        Estimate remaining travel time (seconds) along a path from current_edge.
        Uses edge mean speed and occupancy with a congestion penalty.
        """
        if current_edge not in path:
            return float("inf")

        start_idx = path.index(current_edge)
        eta = 0.0

        for edge in path[start_idx:]:
            edge_length = float(self.connection_info.edge_length_dict.get(edge, 0.0))
            try:
                edge_speed = float(traci.edge.getLastStepMeanSpeed(edge))
            except Exception:
                edge_speed = 0.0
            try:
                occ_raw = float(traci.edge.getLastStepOccupancy(edge))
            except Exception:
                occ_raw = 0.0

            occ_ratio = (occ_raw / 100.0) if occ_raw > 1.0 else occ_raw

            # Stronger penalty so blocked edges are not underestimated.
            congestion_penalty = 1.0 + max(0.0, occ_ratio - 0.10) * 5.0
            effective_speed = max(0.3, edge_speed / congestion_penalty)

            if edge_length > 0:
                eta += edge_length / effective_speed

        return eta

    def make_decisions(self, vehicles, connection_info):
        # Start with A* decisions for all vehicles
        local_targets = super().make_decisions(vehicles, connection_info)

        for vehicle in vehicles:
            if str(vehicle.vehicle_id) != self.hero_vehicle_id:
                continue

            direct_eta = self._estimate_path_time(vehicle.current_edge, self.direct_path)
            alt_eta = self._estimate_path_time(vehicle.current_edge, self.alternative_path)
            direct_congested = self._check_route_congestion(vehicle.current_edge, self.direct_path)
            alt_congested = self._check_route_congestion(vehicle.current_edge, self.alternative_path)
            now = float(traci.simulation.getTime())
            can_switch = (self._last_switch_time < 0) or ((now - self._last_switch_time) >= self._switch_cooldown_s)
            required_gain = max(1.0, 0.05 * min(direct_eta, alt_eta))

            if can_switch:
                # Primary trigger: direct route is congested while alternate is not.
                if self.hero_current_plan == "direct" and direct_congested and not alt_congested:
                    self.reroute_decision_direct_eta = direct_eta
                    self.reroute_decision_alternative_eta = alt_eta
                    self.hero_current_plan = "alternative"
                    self._last_switch_time = now
                    print(f"[DYNAMIC TURN] direct -> alternative at {vehicle.current_edge} | reason=congestion")
                elif self.hero_current_plan == "direct" and (alt_eta + required_gain < direct_eta):
                    self.reroute_decision_direct_eta = direct_eta
                    self.reroute_decision_alternative_eta = alt_eta
                    self.hero_current_plan = "alternative"
                    self._last_switch_time = now
                    print(f"[DYNAMIC TURN] direct -> alternative at {vehicle.current_edge} | gain={direct_eta - alt_eta:.2f}s")
                elif self.hero_current_plan == "alternative" and (direct_eta + required_gain < alt_eta) and not direct_congested:
                    self.hero_current_plan = "direct"
                    self._last_switch_time = now
                    print(f"[DYNAMIC TURN] alternative -> direct at {vehicle.current_edge} | gain={alt_eta - direct_eta:.2f}s")
            
            # Use the current active plan
            active_path = self.alternative_path if self.hero_current_plan == "alternative" else self.direct_path
            decision_list = self._edge_path_to_decisions(vehicle.current_edge, active_path)

            target = None
            if len(decision_list) > 0:
                target = self.compute_local_target(decision_list, vehicle)

            # Fallback should only be used when suffix-path routing is unavailable.
            # Overriding a valid target while stopped at lights can cause terminal loops.
            should_fallback = (target is None)
            if should_fallback:
                dynamic_target = self._choose_congestion_aware_target(vehicle, vehicle.destination)
                if dynamic_target is not None:
                    target = dynamic_target

            if target is not None:
                local_targets[vehicle.vehicle_id] = target

        return local_targets
