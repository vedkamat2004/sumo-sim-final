from controller.AStarController import AStarPolicy


class FixedPathPolicy(AStarPolicy):
    """
    Policy that enforces a fixed edge path for one designated vehicle.
    All non-hero vehicles are routed by AStarPolicy.
    """

    def __init__(self, connection_info, hero_vehicle_id, hero_edge_path):
        super().__init__(connection_info)
        self.hero_vehicle_id = str(hero_vehicle_id)
        self.hero_edge_path = list(hero_edge_path)

    def _edge_path_to_decisions(self, current_edge):
        """
        Converts a suffix of hero_edge_path into SUMO direction decisions.
        Returns an empty list when no valid continuation exists.
        """
        if current_edge not in self.hero_edge_path:
            return []

        current_index = self.hero_edge_path.index(current_edge)
        decisions = []

        for i in range(current_index, len(self.hero_edge_path) - 1):
            from_edge = self.hero_edge_path[i]
            to_edge = self.hero_edge_path[i + 1]
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

    def make_decisions(self, vehicles, connection_info):
        # Start with A* decisions so non-hero vehicles behave normally.
        local_targets = super().make_decisions(vehicles, connection_info)

        for vehicle in vehicles:
            if str(vehicle.vehicle_id) != self.hero_vehicle_id:
                continue

            decision_list = self._edge_path_to_decisions(vehicle.current_edge)
            if len(decision_list) == 0:
                # If path cannot be continued, keep A* fallback.
                continue

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets
