from pathlib import Path
from typing import List, Tuple

from tabulate import tabulate

from drone_sim.models.collision import CollisionRecord
from drone_sim.models.drone import Drone


class ReportGenerator:
    """Helper class to generate formatted simulation reports"""
    
    def __init__(self, output_dir: Path):
        self.output_dir = output_dir
        self.output_dir.mkdir(exist_ok=True, parents=True)

    @staticmethod
    def generate_report(
        city_name: str,
        simulation_time: float,
        total_flights: int,
        successful_flights: int,
        avg_travel_time: float,
        drone_collisions: List[Tuple[int, int, float]],
        building_collisions: List[Tuple[int, int, float, float, float, float]],
        building_count: int,
        battery_depletions: int = 0
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
Battery Depletions: {battery_depletions}
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

    @staticmethod
    def _count_successful_flights(drones: List[Drone]) -> int:
        return sum(1 for drone in drones if drone.successful)

    @staticmethod
    def _calculate_avg_travel_time(drones: List[Drone]) -> float:
        successful_times = [d.travel_time for d in drones if d.successful]
        return sum(successful_times) / len(successful_times) if successful_times else 0.0

    def generate_and_save_report(self, drones: List[Drone], city_name: str,
                                simulation_time: float, 
                                collisions: List[CollisionRecord],
                                building_count: int) -> Tuple[Path, str]:
        """Generate simulation report and save to file"""
        
        report = self.generate_report(
            city_name=city_name,
            simulation_time=simulation_time,
            total_flights=len(drones),
            successful_flights=self._count_successful_flights(drones),
            avg_travel_time=self._calculate_avg_travel_time(drones),
            drone_collisions=[(c.drone_id, c.other_id, c.timestamp) for c in collisions if c.collision_type == 'drone'],
            building_collisions=[(c.drone_id, c.other_id, c.timestamp, *c.position) for c in collisions if c.collision_type == 'building'],
            building_count=building_count
        )

        # Save report
        report_path = self.output_dir / f"simulation_report_{simulation_time:.1f}s.txt"
        with open(report_path, 'w') as f:
            f.write(report)

        return (report_path, report)

    def generate_summary_report(self, collisions):
        report = self.generate_report(
            total_collisions=len(collisions),
            # ... other params ...
        )
        return report 