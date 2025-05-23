import sys
import types

# Stub numpy before importing modules that rely on it
if 'numpy' not in sys.modules:
    numpy_stub = types.ModuleType('numpy')
    numpy_stub.array = lambda *args, **kwargs: []
    numpy_stub.ndarray = object  # minimal placeholder for type hints
    sys.modules['numpy'] = numpy_stub

from drone_sim.models.city_data import CityData

class DummyBuilding:
    def to_dict(self):
        return {}

    def to_geojson(self):
        return {}


def test_city_data_to_dict_keys():
    city = CityData(name="TestCity", buildings=[DummyBuilding()], dimensions=(100, 100, 50))
    city.building_density_per_km2 = 15
    city.avg_height_m = 30.0
    city.population_density_per_km2 = 1000
    city.takeoff_locations = 3

    result = city.to_dict()

    required_keys = {
        "name",
        "building_density_per_km2",
        "avg_height_m",
        "population_density_per_km2",
        "takeoff_locations",
        "buildings",
        "dimensions",
    }
    assert required_keys.issubset(result)
