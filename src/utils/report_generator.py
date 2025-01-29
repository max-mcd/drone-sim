from typing import List, Tuple

from tabulate import tabulate


class ReportGenerator:
    """Helper class to generate formatted simulation reports"""
    
    @staticmethod
    def generate_report(
        city_name: str,
        simulation_time: float,
        total_flights: int,
        successful_flights: int,
        avg_travel_time: float,
        drone_collisions: List[Tuple[int, int, float]],
        building_collisions: List[Tuple[int, int, float, float, float, float]],
        building_count: int
    ) -> str:
        """Generate a formatted simulation report."""
        return f"""
+------------------------------------------------------------------------------------------------------------+
| ____                         _____ _ _       _     _     ____  _                 _       _   _             |
||  _ \ _ __ ___  _ __   ___  |  ___| (_) __ _| |__ | |_  / ___|(_)_ __ ___  _   _| | __ _| |_(_) ___  _ __  |
|| | | | '__/ _ \| '_ \ / _ \ | |_  | | |/ _` | '_ \| __| \___ \| | '_ ` _ \| | | | |/ _` | __| |/ _ \| '_ \ |
|| |_| | | | (_) | | | |  __/ |  _| | | | (_| | | | | |_   ___) | | | | | | | |_| | | (_| | |_| | (_) | | | ||
||____/|_|  \___/|_| |_|\___| |_|   |_|_|\__, |_| |_|\__| |____/|_|_| |_| |_|\__,_|_|\__,_|\__|_|\___/|_| |_||
|                                        |___/                                                               |
+------------------------------------------------------------------------------------------------------------+

🌆 City: {city_name}
🏢 Buildings: {building_count}
🕒 Simulation Time: {simulation_time:.1f} seconds

📊 FLIGHT STATISTICS
═══════════════════
Total Flights: {total_flights}
Successful Flights: {successful_flights}
Success Rate: {(successful_flights/total_flights)*100:.1f}%
Average Travel Time: {avg_travel_time:.1f} seconds

💥 COLLISION SUMMARY
══════════════════
Drone-Drone Collisions: {len(drone_collisions)}
Drone-Building Collisions: {len(building_collisions)}
Total Collisions: {len(drone_collisions) + len(building_collisions)}

🚨 DETAILED COLLISION LOG
═══════════════════════

DRONE COLLISIONS:
{ReportGenerator._format_drone_collisions(drone_collisions)}

BUILDING COLLISIONS:
{ReportGenerator._format_building_collisions(building_collisions)}
"""

    @staticmethod
    def _format_drone_collisions(collisions: List[Tuple[int, int, float]]) -> str:
        if not collisions:
            return tabulate(
                [["None", "None", "--"]], 
                headers=["Drone 1", "Drone 2", "Time"],
                tablefmt="simple_grid"
            )
        
        rows = [[d1, d2, f"{t:.1f}s"] for d1, d2, t in collisions]
        return tabulate(
            rows,
            headers=["Drone 1", "Drone 2", "Time"],
            tablefmt="simple_grid",
            numalign="center"
        )

    @staticmethod
    def _format_building_collisions(
        collisions: List[Tuple[int, int, float, float, float, float]]
    ) -> str:
        if not collisions:
            return tabulate(
                [["None", "None", "--", "--", "--", "--"]], 
                headers=["Drone", "Building", "Time", "X", "Y", "Z"],
                tablefmt="simple_grid"
            )
        
        rows = [
            [d, b, f"{t:.1f}s", f"{x:.1f}", f"{y:.1f}", f"{z:.1f}"] 
            for d, b, t, x, y, z in collisions
        ]
        return tabulate(
            rows,
            headers=["Drone", "Building", "Time", "X", "Y", "Z"],
            tablefmt="simple_grid",
            numalign="center"
        ) 