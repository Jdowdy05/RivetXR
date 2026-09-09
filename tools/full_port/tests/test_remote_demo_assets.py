"""Both new remote representations must actually be present in the APK."""
import unittest


class RemoteDemoAssetTests(unittest.TestCase):
    def payloads(self):
        from tools.remote_scene.demo import make_demo
        from tools.remote_scene.gimbal_demo import make_gimbal_demo
        from tools.remote_scene.protocol import pack_packet
        class Payload:
            def __init__(self):
                self.data={f'assets/remote_scene/{name}':pack_packet(make_demo(geometry=mode)[0])
                    for name,mode in (('demo.rscn','points'),('demo-prepared.rscn','prepared'),('demo-unprepared.rscn','unprepared'))}
                self.data.update({f'assets/remote_scene/demo-rgb-{mode}.rscn':pack_packet(make_gimbal_demo(geometry=mode)[0])
                    for mode in ('prepared','unprepared')})
            def read(self,name):return self.data[name]
        return Payload()

    def test_all_modes_match_current_generator(self):
        from tools.full_port.verify_apk import verify_remote_demos
        result=verify_remote_demos(self.payloads())
        self.assertEqual(set(result),{'points','prepared','unprepared','rgb-prepared','rgb-unprepared'})

    def test_missing_or_wrong_representation_cannot_pass(self):
        from tools.full_port.verify_apk import verify_remote_demos
        payload=self.payloads();del payload.data['assets/remote_scene/demo-prepared.rscn']
        with self.assertRaises((KeyError,ValueError)):verify_remote_demos(payload)
        payload=self.payloads();payload.data['assets/remote_scene/demo-prepared.rscn']=payload.data['assets/remote_scene/demo-unprepared.rscn']
        with self.assertRaises(ValueError):verify_remote_demos(payload)

    def test_rgb_assets_cannot_be_missing_or_interchanged(self):
        from tools.full_port.verify_apk import verify_remote_demos
        payload=self.payloads();del payload.data['assets/remote_scene/demo-rgb-prepared.rscn']
        with self.assertRaises(KeyError):verify_remote_demos(payload)
        payload=self.payloads();payload.data['assets/remote_scene/demo-rgb-prepared.rscn']=payload.data['assets/remote_scene/demo-rgb-unprepared.rscn']
        with self.assertRaises(ValueError):verify_remote_demos(payload)


if __name__=='__main__':unittest.main()
