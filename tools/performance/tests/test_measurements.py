import unittest
from tools.performance.measurements import evaluate, health


class MeasurementTests(unittest.TestCase):
    def native(self):
        return dict(phase='complete', finite=True, passed=True, requested_physics_hz=200,
                    wall_seconds=30., simulation_seconds=30., successful_steps=6000,
                    dropped_wall_seconds=0., frame_count=2700, refresh_hz=90.,
                    physics_timing=dict(count=6000, mean_us=3000, p99_us=4000))

    def test_average_capacity_is_not_a_sustainable_rate(self):
        data=self.native();data.update(requested_physics_hz=1000,simulation_seconds=6.,dropped_wall_seconds=24.)
        result=evaluate(data)
        self.assertEqual(result['achieved_hz'],200)
        self.assertFalse(result['physics_sustainable'])

    def test_tail_headroom_and_realtime_are_distinct(self):
        data=self.native();data['physics_timing']['p99_us']=5100
        result=evaluate(data)
        self.assertTrue(result['physics_sustainable'])
        self.assertFalse(result['recommended_candidate'])
        self.assertIsNone(result['external_xr'])

    def test_faulted_and_inconsistent_records_never_pass(self):
        for change in ({'finite':False},{'phase':'failed'},{'successful_steps':5999}):
            data=self.native();data.update(change)
            self.assertFalse(evaluate(data)['physics_sustainable'])

    def test_missing_thermal_or_memory_is_unavailable(self):
        result=health('level: 38\ntemperature: 380\nAC powered: true','Thermal Status: 0','unavailable')
        self.assertEqual(result['battery_celsius'],38)
        self.assertEqual(result['thermal_status'],0)
        self.assertIsNone(result['cpu_max_celsius'])
        self.assertIsNone(result['pss_kb'])

    def test_only_temperature_sensor_types_are_aggregated(self):
        result=health('', 'Current temperatures from HAL:\nTemperature{mValue=60.0, mType=0, mName=cpu, mStatus=0}\n'
                      'Temperature{mValue=560, mType=7, mName=current, mStatus=0}', '')
        self.assertEqual(result['cpu_max_celsius'],60)
        self.assertIsNone(result['gpu_max_celsius'])

    def test_missing_charging_data_remains_unknown(self):
        for text in ('', 'Permission denied', 'level: 38\ntemperature: 380'):
            with self.subTest(text=text):
                self.assertIsNone(health(text, '', '')['charging'])

    def test_explicit_unpowered_state_is_false(self):
        text = 'AC powered: false\nUSB powered: false\nWireless powered: false'
        self.assertIs(health(text, '', '')['charging'], False)

    def test_any_powered_source_reports_charging(self):
        text = 'AC powered: false\nUSB powered: true\nWireless powered: false'
        self.assertIs(health(text, '', '')['charging'], True)


if __name__=='__main__':unittest.main()
