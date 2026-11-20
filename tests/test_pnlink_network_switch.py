"""Network migration safety: ordering, preserved internal NIC, rollback and gates."""
import copy
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'pnlink_network_switch', Path(__file__).resolve().parents[1] / 'scripts/switch_pnlink_network.py')
switch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(switch)


def profiles():
    return {
        'eth0': dict(uuid='wired-uuid', name='example-wired', type='802-3-ethernet', ssid='',
                     addresses=['192.168.123.80/24', switch.CIDR], routes=[], autoconnect=True,
                     protected=dict(method='manual', gateway='192.168.123.1', **{'route-metric': -1})),
        'wlan0': dict(uuid='wifi-uuid', name='example-wifi', type='802-11-wireless', ssid='example-wifi',
                      addresses=[], routes=[], autoconnect=True,
                      protected=dict(method='auto', **{'route-metric': -1})),
    }


class FakeNetwork:
    def __init__(self, initial=None):
        self.saved = copy.deepcopy(initial or profiles())
        self.history = []
        self.fail_after_write = None
        self.announcements = []

    def snapshot(self):
        return copy.deepcopy(self.saved)

    def apply_profile(self, iface, wanted):
        if self.saved[iface] == wanted:
            return
        self.saved[iface] = copy.deepcopy(wanted)
        self.history.append(self.snapshot())
        if self.fail_after_write == len(self.history):
            self.fail_after_write = None
            raise RuntimeError('simulated reapply failure after persistent write')

    def addresses(self):
        rows = []
        for iface, profile in self.saved.items():
            addresses = profile['addresses'] + (['192.0.2.91/24'] if iface == 'wlan0' else [])
            rows.append(dict(ifname=iface, addr_info=[dict(local=a.split('/')[0], prefixlen=int(a.split('/')[1])) for a in addresses]))
        return rows

    def route(self, destination, source=None):
        if destination == switch.ROBOT:
            return dict(dev='eth0', prefsrc='192.168.123.80')
        owners = switch.address_owners(self)
        return dict(dev=owners[0] if len(owners) == 1 else 'wrong', prefsrc=switch.LOCAL)

    def arp(self, iface, address, source='0.0.0.0', announce=False):
        if announce:
            self.announcements.append(iface)
        return ['02:00:00:00:00:01']


class NetworkSwitchTest(unittest.TestCase):
    def assert_safe_history(self, network):
        for state in network.history:
            self.assertIn('192.168.123.80/24', state['eth0']['addresses'])
            self.assertLessEqual(sum(switch.CIDR in p['addresses'] for p in state.values()), 1)
            self.assertEqual(state['eth0']['protected'], profiles()['eth0']['protected'])
            self.assertEqual(state['wlan0']['protected'], profiles()['wlan0']['protected'])

    def test_wired_wifi_roundtrip_preserves_dhcp_internal_and_other_routes(self):
        original = profiles()
        original['eth0']['routes'] = [dict(dest='192.168.55.0', prefix=24, **{'next-hop': '192.168.123.1', 'metric': 200})]
        network = FakeNetwork(original)
        wifi = switch.desired_profiles(network.snapshot(), 'wlan0')
        switch.transaction(network, original, wifi, lambda: switch.verify_profiles(network, wifi))
        self.assertEqual(switch.address_owners(network), ['wlan0'])
        self.assertIn('192.0.2.91', str(network.addresses()))
        self.assertEqual(network.saved['eth0']['routes'], original['eth0']['routes'])
        wired = switch.desired_profiles(network.snapshot(), 'eth0')
        switch.transaction(network, wifi, wired, lambda: switch.verify_profiles(network, wired))
        self.assertEqual(switch.address_owners(network), ['eth0'])
        self.assertEqual(network.saved['wlan0']['addresses'], [])
        self.assert_safe_history(network)

    def test_restore_returns_exact_original_state_without_managed_route(self):
        original = profiles()
        network = FakeNetwork(original)
        switch.transition(network, switch.desired_profiles(original, 'wlan0'))
        switch.transition(network, original)
        switch.verify_profiles(network, original)
        self.assertEqual(network.saved, original)
        self.assert_safe_history(network)

    def test_restore_supports_initial_state_with_no_receive_address(self):
        original = profiles()
        original['eth0']['addresses'] = ['192.168.123.80/24']
        network = FakeNetwork(original)
        switch.transition(network, switch.desired_profiles(original, 'wlan0'))
        switch.transition(network, original)
        switch.verify_profiles(network, original)
        self.assertEqual(switch.address_owners(network), [])

    def test_persistent_write_then_live_reapply_failure_rolls_back(self):
        original = profiles()
        network = FakeNetwork(original)
        network.fail_after_write = 2
        with self.assertRaises(switch.SwitchFailure) as caught:
            switch.transaction(network, original, switch.desired_profiles(original, 'wlan0'), lambda: None)
        self.assertIsNone(caught.exception.rollback_error)
        self.assertEqual(network.saved, original)
        self.assertEqual(network.announcements, ['eth0'])
        self.assert_safe_history(network)

    def test_postcheck_failure_restores_old_address_owner(self):
        original = profiles()
        network = FakeNetwork(original)
        def failed_link():
            raise RuntimeError('device not reachable on Wi-Fi')
        with self.assertRaises(switch.SwitchFailure):
            switch.transaction(network, original, switch.desired_profiles(original, 'wlan0'), failed_link)
        self.assertEqual(network.saved, original)
        self.assert_safe_history(network)

    def test_rollback_failure_is_reported(self):
        network = FakeNetwork()
        def fail_then_break_rollback():
            network.fail_after_write = len(network.history) + 1
            raise RuntimeError('failed device check')
        with self.assertRaises(switch.SwitchFailure) as caught:
            switch.transaction(network, profiles(), switch.desired_profiles(profiles(), 'wlan0'), fail_then_break_rollback)
        self.assertIn('simulated reapply failure', caught.exception.rollback_error)

    def test_repeated_target_is_idempotent(self):
        target = switch.desired_profiles(profiles(), 'wlan0')
        network = FakeNetwork(target)
        switch.transition(network, switch.desired_profiles(target, 'wlan0'))
        self.assertEqual(network.history, [])

    def test_duplicate_ip_internal_conflict_and_custom_device_route_are_rejected(self):
        for kind in ('duplicate', 'internal', 'custom-route', 'wrong-mask', 'lost-internal', 'wifi-manual'):
            with self.subTest(kind=kind):
                value = profiles()
                if kind == 'duplicate':value['wlan0']['addresses'] = [switch.CIDR]
                if kind == 'internal':value['wlan0']['addresses'] = ['192.168.123.81/24']
                if kind == 'custom-route':value['eth0']['routes'] = [dict(dest=switch.SERVER, prefix=32, metric=7)]
                if kind == 'wrong-mask':value['eth0']['addresses'][1] = switch.LOCAL + '/16'
                if kind == 'lost-internal':value['eth0']['addresses'] = [switch.CIDR]
                if kind == 'wifi-manual':value['wlan0']['protected']['method'] = 'manual'
                with self.assertRaises(RuntimeError):switch.validate_profiles(value)

    def test_site_override_moves_old_and_new_addresses_transactionally(self):
        original = profiles()
        old = switch.DEFAULT_SITE
        original['eth0']['routes'] = [dict(dest=old.server, prefix=32, metric=50)]
        new = switch.Site('172.20.4.50/23', '172.20.5.20', old)
        network = FakeNetwork(original)
        wanted = switch.desired_profiles(original, 'wlan0', new)
        switch.transaction(network, original, wanted,
                           lambda: switch.verify_profiles(network, wanted, new), new)
        self.assertEqual(network.saved['eth0']['addresses'], ['192.168.123.80/24'])
        self.assertEqual(network.saved['eth0']['routes'], [])
        self.assertEqual(network.saved['wlan0']['addresses'], [new.cidr])
        switch.transaction(network, wanted, original,
                           lambda: switch.verify_profiles(network, original, new), new)
        self.assertEqual(network.saved, original)

    def test_first_wired_setup_can_run_without_a_wifi_profile(self):
        original = {'eth0': profiles()['eth0']}
        original['eth0']['addresses'] = ['192.168.123.80/24']
        switch.validate_profiles(original)
        wanted = switch.desired_profiles(original, 'eth0')
        network = FakeNetwork(original)
        switch.transition(network, wanted)
        switch.verify_profiles(network, wanted)
        self.assertEqual(switch.address_owners(network), ['eth0'])

    def test_site_cannot_overlap_internal_dds_or_use_network_address(self):
        for cidr, box in (('192.168.123.120/24', '192.168.123.43'),
                          ('10.7.0.0/24', '10.7.0.33'),
                          ('10.7.0.20/24', '10.8.0.33')):
            with self.subTest(cidr=cidr), self.assertRaises(ValueError):
                switch.Site(cidr, box)

    def test_active_policy_or_hand_control_is_rejected(self):
        idle = dict(mode='onboard_real', robot_id='unitree_g1', state='stopped', pids=[])
        switch.require_idle(idle)
        for update in (dict(state='playing'), dict(pids=[123]), dict(mode_switch_pending=True),
                       dict(pnlink_command_pending=True), dict(pico_hands=dict(active=True)),
                       dict(pnlink_hands=dict(worker=dict(sides=dict(right=dict(commanding=True)))))):
            with self.subTest(update=update):
                with self.assertRaises(RuntimeError):switch.require_idle(dict(idle, **update))

    def test_wrong_live_route_is_rejected_even_with_correct_saved_profiles(self):
        target = switch.desired_profiles(profiles(), 'wlan0')
        network = FakeNetwork(target)
        network.route = lambda *args: dict(dev='wlan0', prefsrc='192.168.123.80')
        with self.assertRaisesRegex(RuntimeError, '内部'):
            switch.verify_profiles(network, target)


if __name__ == '__main__':
    unittest.main()
