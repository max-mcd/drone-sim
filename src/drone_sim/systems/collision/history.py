from typing import List

from drone_sim.models.collision import CollisionRecord


class CollisionHistory:
    def __init__(self):
        self.records: List[CollisionRecord] = []
        
    def add_record(self, record: CollisionRecord):
        self.records.append(record)
        
    def get_summary(self) -> dict:
        return {
            'total': len(self.records),
            'avoided': sum(r.avoided for r in self.records),
            'by_severity': self._severity_distribution()
        }
    
    def _severity_distribution(self) -> dict:
        return {
            'low': sum(0 < r.severity < 0.3 for r in self.records),
            'medium': sum(0.3 <= r.severity < 0.7 for r in self.records),
            'high': sum(r.severity >= 0.7 for r in self.records)
        }
    
    def get_recent_records(self, max_records=50):
        """Get most recent collision records"""
        return self.records[-max_records:]  # Return slice of recent records 