try:
    import usb_hid
except:
    pass

from micropython import const

from kmk.keys import FIRST_KMK_INTERNAL_KEY, ConsumerKey, ModifierKey
from storage import getmount

try:
    from adafruit_ble import BLERadio
    from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
    from adafruit_ble.services.standard.hid import HIDService
except ImportError:
    # BLE not supported on this platform
    pass

try:
    import board
    import busio
    import digitalio
except:
    # BluefruitSPI library missing
    pass
from kmk.drivers.bluefruitspi import BluefruitSPI


class HIDModes:
    NOOP = 0  # currently unused; for testing?
    USB = 1
    BLE = 2
    BLUEFRUITSPI = 4

    ALL_MODES = (NOOP, USB, BLE, BLUEFRUITSPI)


class HIDReportTypes:
    KEYBOARD = 1
    MOUSE = 2
    CONSUMER = 3
    SYSCONTROL = 4


class HIDUsage:
    KEYBOARD = 0x06
    MOUSE = 0x02
    CONSUMER = 0x01
    SYSCONTROL = 0x80


class HIDUsagePage:
    CONSUMER = 0x0C
    KEYBOARD = MOUSE = SYSCONTROL = 0x01


HID_REPORT_SIZES = {
    HIDReportTypes.KEYBOARD: 8,
    HIDReportTypes.MOUSE: 4,
    HIDReportTypes.CONSUMER: 2,
    HIDReportTypes.SYSCONTROL: 8,  # TODO find the correct value for this
}


class AbstractHID:
    REPORT_BYTES = 8

    def __init__(self):
        self._evt = bytearray(self.REPORT_BYTES)
        self.report_device = memoryview(self._evt)[0:1]
        self.report_device[0] = HIDReportTypes.KEYBOARD

        # Landmine alert for HIDReportTypes.KEYBOARD: byte index 1 of this view
        # is "reserved" and evidently (mostly?) unused. However, other modes (or
        # at least consumer, so far) will use this byte, which is the main reason
        # this view exists. For KEYBOARD, use report_mods and report_non_mods
        self.report_keys = memoryview(self._evt)[1:]

        self.report_mods = memoryview(self._evt)[1:2]
        self.report_non_mods = memoryview(self._evt)[3:]

        self.post_init()

    def __repr__(self):
        return '{}(REPORT_BYTES={})'.format(self.__class__.__name__, self.REPORT_BYTES)

    def post_init(self):
        pass

    def create_report(self, keys_pressed):
        self.clear_all()

        consumer_key = None
        for key in keys_pressed:
            if isinstance(key, ConsumerKey):
                consumer_key = key
                break

        reporting_device = self.report_device[0]
        needed_reporting_device = HIDReportTypes.KEYBOARD

        if consumer_key:
            needed_reporting_device = HIDReportTypes.CONSUMER

        if reporting_device != needed_reporting_device:
            # If we are about to change reporting devices, release
            # all keys and close our proverbial tab on the existing
            # device, or keys will get stuck (mostly when releasing
            # media/consumer keys)
            self.send()

        self.report_device[0] = needed_reporting_device

        if consumer_key:
            self.add_key(consumer_key)
        else:
            for key in keys_pressed:
                if key.code >= FIRST_KMK_INTERNAL_KEY:
                    continue

                if isinstance(key, ModifierKey):
                    self.add_modifier(key)
                else:
                    self.add_key(key)

                    if key.has_modifiers:
                        for mod in key.has_modifiers:
                            self.add_modifier(mod)

        return self

    def hid_send(self, evt):
        # Don't raise a NotImplementedError so this can serve as our "dummy" HID
        # when MCU/board doesn't define one to use (which should almost always be
        # the CircuitPython-targeting one, except when unit testing or doing
        # something truly bizarre. This will likely change eventually when Bluetooth
        # is added)
        print(evt)

    def send(self):
        self.hid_send(self._evt)

        return self

    def clear_all(self):
        for idx, _ in enumerate(self.report_keys):
            self.report_keys[idx] = 0x00

        return self

    def clear_non_modifiers(self):
        for idx, _ in enumerate(self.report_non_mods):
            self.report_non_mods[idx] = 0x00

        return self

    def add_modifier(self, modifier):
        if isinstance(modifier, ModifierKey):
            if modifier.code == ModifierKey.FAKE_CODE:
                for mod in modifier.has_modifiers:
                    self.report_mods[0] |= mod
            else:
                self.report_mods[0] |= modifier.code
        else:
            self.report_mods[0] |= modifier

        return self

    def remove_modifier(self, modifier):
        if isinstance(modifier, ModifierKey):
            if modifier.code == ModifierKey.FAKE_CODE:
                for mod in modifier.has_modifiers:
                    self.report_mods[0] ^= mod
            else:
                self.report_mods[0] ^= modifier.code
        else:
            self.report_mods[0] ^= modifier

        return self

    def add_key(self, key):
        # Try to find the first empty slot in the key report, and fill it
        placed = False

        where_to_place = self.report_non_mods

        if self.report_device[0] == HIDReportTypes.CONSUMER:
            where_to_place = self.report_keys

        for idx, _ in enumerate(where_to_place):
            if where_to_place[idx] == 0x00:
                where_to_place[idx] = key.code
                placed = True
                break

        if not placed:
            # TODO what do we do here?......
            pass

        return self

    def remove_key(self, key):
        where_to_place = self.report_non_mods

        if self.report_device[0] == HIDReportTypes.CONSUMER:
            where_to_place = self.report_keys

        for idx, _ in enumerate(where_to_place):
            if where_to_place[idx] == key.code:
                where_to_place[idx] = 0x00

        return self


class USBHID(AbstractHID):
    REPORT_BYTES = 9

    def post_init(self):
        self.devices = {}

        for device in usb_hid.devices:
            us = device.usage
            up = device.usage_page

            if up == HIDUsagePage.CONSUMER and us == HIDUsage.CONSUMER:
                self.devices[HIDReportTypes.CONSUMER] = device
                continue

            if up == HIDUsagePage.KEYBOARD and us == HIDUsage.KEYBOARD:
                self.devices[HIDReportTypes.KEYBOARD] = device
                continue

            if up == HIDUsagePage.MOUSE and us == HIDUsage.MOUSE:
                self.devices[HIDReportTypes.MOUSE] = device
                continue

            if up == HIDUsagePage.SYSCONTROL and us == HIDUsage.SYSCONTROL:
                self.devices[HIDReportTypes.SYSCONTROL] = device
                continue

    def hid_send(self, evt):
        # int, can be looked up in HIDReportTypes
        reporting_device_const = self.report_device[0]

        return self.devices[reporting_device_const].send_report(
            evt[1 : HID_REPORT_SIZES[reporting_device_const] + 1]
        )


class BLEHID(AbstractHID):
    BLE_APPEARANCE_HID_KEYBOARD = const(961)
    # Hardcoded in CPy
    MAX_CONNECTIONS = const(2)

    def post_init(self, ble_name=str(getmount('/').label), **kwargs):
        self.conn_id = -1

        self.ble = BLERadio()
        self.ble.name = ble_name
        self.hid = HIDService()
        self.hid.protocol_mode = 0  # Boot protocol

        # Security-wise this is not right. While you're away someone turns
        # on your keyboard and they can pair with it nice and clean and then
        # listen to keystrokes.
        # On the other hand we don't have LESC so it's like shouting your
        # keystrokes in the air
        if not self.ble.connected or not self.hid.devices:
            self.start_advertising()

        self.conn_id = 0

    @property
    def devices(self):
        '''Search through the provided list of devices to find the ones with the
            send_report attribute.'''
        if not self.ble.connected:
            return []

        result = []
        # Security issue:
        # This introduces a race condition. Let's say you have 2 active
        # connections: Alice and Bob - Alice is connection 1 and Bob 2.
        # Now Chuck who has already paired with the device in the past
        # (this assumption is needed only in the case of LESC)
        # wants to gather the keystrokes you send to Alice. You have
        # selected right now to talk to Alice (1) and you're typing a secret.
        # If Chuck kicks Alice off and is quick enough to connect to you,
        # which means quicker than the running interval of this function,
        # he'll be earlier in the `self.hid.devices` so will take over the
        # selected 1 position in the resulted array.
        # If no LESC is in place, Chuck can sniff the keystrokes anyway
        for device in self.hid.devices:
            if hasattr(device, 'send_report'):
                result.append(device)

        return result

    def _check_connection(self):
        devices = self.devices
        if not devices:
            return False

        if self.conn_id >= len(devices):
            self.conn_id = len(devices) - 1

        if self.conn_id < 0:
            return False

        if not devices[self.conn_id]:
            return False

        return True

    def hid_send(self, evt):
        if not self._check_connection():
            return

        device = self.devices[self.conn_id]

        while len(evt) < len(device._characteristic.value) + 1:
            evt.append(0)

        return device.send_report(evt[1:])

    def clear_bonds(self):
        import _bleio

        _bleio.adapter.erase_bonding()

    def next_connection(self):
        self.conn_id = (self.conn_id + 1) % len(self.devices)

    def previous_connection(self):
        self.conn_id = (self.conn_id - 1) % len(self.devices)

    def start_advertising(self):
        advertisement = ProvideServicesAdvertisement(self.hid)
        advertisement.appearance = self.BLE_APPEARANCE_HID_KEYBOARD

        self.ble.start_advertising(advertisement)

    def stop_advertising(self):
        self.ble.stop_advertising()

class BluefruitSPIHID(AbstractHID):
    REPORT_BYTES = 9
    def __init__(self, cs, irq, rst):
        self.cs = digitalio.DigitalInOut(cs)
        self.irq = digitalio.DigitalInOut(irq)
        self.rst = digitalio.DigitalInOut(rst)
        super(BluefruitSPIHID, self).__init__()

    def post_init(self, ble_name=str(getmount('/').label), **kwargs):
        print("Initializing bluefruit")
        self.spi = busio.SPI(board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        self.bluefruit = BluefruitSPI(self.spi, self.cs, self.irq, self.rst, debug=False)

        self.bluefruit.init()
        self.bluefruit.command_check_OK(b"AT+FACTORYRESET", delay=1)
        self.bluefruit.command(b'ATE=0')
        self.bluefruit.command(f'AT+GAPDEVNAME={ble_name}'.encode())
        self.bluefruit.command(b'AT+BLEHIDEN=1')
        self.bluefruit.command(b'AT+GAPINTERVALS=10,30,,')
        print("Resetting bluefruit")
        self.bluefruit.command(b'ATZ')
        print("bluefruit ready")

    def hid_send(self, evt):
        self.bluefruit.send_keyboard_code(evt[1:])

    def after_matrix_scan(self):
        self.bluefruit.pop_keyboard_code_queue()