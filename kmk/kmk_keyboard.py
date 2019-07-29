# There's a chance doing preload RAM hacks this late will cause recursion
# errors, but we'll see. I'd rather do it here than require everyone copy-paste
# a line into their keymaps.
import kmk.preload_imports  # isort:skip # NOQA

import busio
import gc

from kmk import led, rgb
from kmk.consts import KMK_RELEASE, LeaderMode, UnicodeMode
from kmk.hid import BLEHID, USBHID, AbstractHID, HIDModes
from kmk.keys import KC
from kmk.kmktime import sleep_ms, ticks_ms
from kmk.matrix import MatrixScanner, intify_coordinate
from kmk.types import TapDanceKeyMeta


class KMKKeyboard:
    #####
    # User-configurable
    debug_enabled = False

    keymap = None
    coord_mapping = None

    row_pins = None
    col_pins = None
    diode_orientation = None
    matrix_scanner = MatrixScanner
    uart_buffer = []

    unicode_mode = UnicodeMode.NOOP
    tap_time = 300
    leader_mode = LeaderMode.TIMEOUT
    leader_dictionary = {}
    leader_timeout = 1000

    # Split config
    extra_data_pin = None
    split_offsets = ()
    split_flip = False
    split_side = None
    split_type = None
    split_master_left = True
    is_master = None
    _uart = None
    uart_flip = True
    uart_pin = None
    uart_timeout = 20

    # RGB config
    rgb_pixel_pin = None
    rgb_config = rgb.rgb_config

    # led config (mono color)
    led_pin = None
    led_config = led.led_config

    #####
    # Internal State
    _keys_pressed = set()
    _coord_keys_pressed = {}
    _leader_pending = None
    _leader_last_len = 0
    _hid_pending = False
    _leader_mode_history = []

    # this should almost always be PREpended to, replaces
    # former use of reversed_active_layers which had pointless
    # overhead (the underlying list was never used anyway)
    _active_layers = [0]

    _start_time = {'lt': None, 'tg': None, 'tt': None, 'lm': None, 'leader': None}
    _timeouts = {}
    _tapping = False
    _tap_dance_counts = {}
    _tap_side_effects = {}

    def __repr__(self):
        return (
            'KMKKeyboard('
            'debug_enabled={} '
            'keymap=truncated '
            'coord_mapping=truncated '
            'row_pins=truncated '
            'col_pins=truncated '
            'diode_orientation={} '
            'matrix_scanner={} '
            'unicode_mode={} '
            'tap_time={} '
            'leader_mode={} '
            'leader_dictionary=truncated '
            'leader_timeout={} '
            'hid_helper={} '
            'extra_data_pin={} '
            'split_offsets={} '
            'split_flip={} '
            'split_side={} '
            'split_type={} '
            'split_master_left={} '
            'is_master={} '
            'uart={} '
            'uart_flip={} '
            'uart_pin={} '
            'keys_pressed={} '
            'coord_keys_pressed={} '
            'leader_pending={} '
            'leader_last_len={} '
            'hid_pending={} '
            'leader_mode_history={} '
            'active_layers={} '
            'start_time={} '
            'timeouts={} '
            'tapping={} '
            'tap_dance_counts={} '
            'tap_side_effects={}'
            ')'
        ).format(
            self.debug_enabled,
            # self.keymap,
            # self.coord_mapping,
            # self.row_pins,
            # self.col_pins,
            self.diode_orientation,
            self.matrix_scanner,
            self.unicode_mode,
            self.tap_time,
            self.leader_mode,
            # self.leader_dictionary,
            self.leader_timeout,
            self.hid_helper.__name__,
            self.extra_data_pin,
            self.split_offsets,
            self.split_flip,
            self.split_side,
            self.split_type,
            self.split_master_left,
            self.is_master,
            self._uart,
            self.uart_flip,
            self.uart_pin,
            # internal state
            self._keys_pressed,
            self._coord_keys_pressed,
            self._leader_pending,
            self._leader_last_len,
            self._hid_pending,
            self._leader_mode_history,
            self._active_layers,
            self._start_time,
            self._timeouts,
            self._tapping,
            self._tap_dance_counts,
            self._tap_side_effects,
        )

    def _print_debug_cycle(self, init=False):
        pre_alloc = gc.mem_alloc()
        pre_free = gc.mem_free()

        if self.debug_enabled:
            if init:
                print('KMKInit(release={})'.format(KMK_RELEASE))

            print(self)
            print(self)
            print(
                'GCStats(pre_alloc={} pre_free={} alloc={} free={})'.format(
                    pre_alloc, pre_free, gc.mem_alloc(), gc.mem_free()
                )
            )

    def _send_hid(self):
        self._hid_helper_inst.create_report(self._keys_pressed).send()
        self._hid_pending = False

    def _handle_matrix_report(self, update=None):
        '''
        Bulk processing of update code for each cycle
        :param update:
        '''
        if update is not None:

            self._on_matrix_changed(update[0], update[1], update[2])

    def _send_to_master(self, update):
        if self.split_master_left:
            update[1] += self.split_offsets[update[0]]
        else:
            update[1] -= self.split_offsets[update[0]]
        if self._uart is not None:
            self._uart.write(update)

    def _receive_from_slave(self):
        if self._uart is not None and self._uart.in_waiting > 0 or self.uart_buffer:
            if self._uart.in_waiting >= 60:
                # This is a dirty hack to prevent crashes in unrealistic cases
                import microcontroller

                microcontroller.reset()

            while self._uart.in_waiting >= 3:
                self.uart_buffer.append(self._uart.read(3))
            if self.uart_buffer:
                update = bytearray(self.uart_buffer.pop(0))

                # Built in debug mode switch
                if update == b'DEB':
                    print(self._uart.readline())
                    return None
                return update

        return None

    def _send_debug(self, message):
        '''
        Prepends DEB and appends a newline to allow debug messages to
        be detected and handled differently than typical keypresses.
        :param message: Debug message
        '''
        if self._uart is not None:
            self._uart.write('DEB')
            self._uart.write(message, '\n')

    #####
    # SPLICE: INTERNAL STATE
    # FIXME CLEAN THIS
    #####

    def _find_key_in_map(self, row, col):
        ic = intify_coordinate(row, col)

        try:
            idx = self.coord_mapping.index(ic)
        except ValueError:
            if self.debug_enabled:
                print(
                    'CoordMappingNotFound(ic={}, row={}, col={})'.format(ic, row, col)
                )

            return None

        for layer in self._active_layers:
            layer_key = self.keymap[layer][idx]

            if not layer_key or layer_key == KC.TRNS:
                continue

            if self.debug_enabled:
                print('KeyResolution(key={})'.format(layer_key))

            return layer_key

    def _on_matrix_changed(self, row, col, is_pressed):
        if self.debug_enabled:
            print('MatrixChange(col={} row={} pressed={})'.format(col, row, is_pressed))

        int_coord = intify_coordinate(row, col)
        kc_changed = self._find_key_in_map(row, col)

        if kc_changed is None:
            print('MatrixUndefinedCoordinate(col={} row={})'.format(col, row))
            return self

        return self._process_key(kc_changed, is_pressed, int_coord, (row, col))

    def _process_key(self, key, is_pressed, coord_int=None, coord_raw=None):
        if self._tapping and not isinstance(key.meta, TapDanceKeyMeta):
            self._process_tap_dance(key, is_pressed)
        else:
            if is_pressed:
                key._on_press(self, coord_int, coord_raw)
            else:
                key._on_release(self, coord_int, coord_raw)

            if self.leader_mode % 2 == 1:
                self._process_leader_mode()

        return self

    def _remove_key(self, keycode):
        self._keys_pressed.discard(keycode)
        return self._process_key(keycode, False)

    def _add_key(self, keycode):
        self._keys_pressed.add(keycode)
        return self._process_key(keycode, True)

    def _tap_key(self, keycode):
        self._add_key(keycode)
        # On the next cycle, we'll remove the key.
        self._set_timeout(False, lambda: self._remove_key(keycode))

        return self

    def _process_tap_dance(self, changed_key, is_pressed):
        if is_pressed:
            if not isinstance(changed_key.meta, TapDanceKeyMeta):
                # If we get here, changed_key is not a TapDanceKey and thus
                # the user kept typing elsewhere (presumably).  End ALL of the
                # currently outstanding tap dance runs.
                for k, v in self._tap_dance_counts.items():
                    if v:
                        self._end_tap_dance(k)

                return self

            if (
                changed_key not in self._tap_dance_counts
                or not self._tap_dance_counts[changed_key]
            ):
                self._tap_dance_counts[changed_key] = 1
                self._set_timeout(
                    self.tap_time, lambda: self._end_tap_dance(changed_key)
                )
                self._tapping = True
            else:
                self._tap_dance_counts[changed_key] += 1

            if changed_key not in self._tap_side_effects:
                self._tap_side_effects[changed_key] = None
        else:
            has_side_effects = self._tap_side_effects[changed_key] is not None
            hit_max_defined_taps = self._tap_dance_counts[changed_key] == len(
                changed_key.codes
            )

            if has_side_effects or hit_max_defined_taps:
                self._end_tap_dance(changed_key)

        return self

    def _end_tap_dance(self, td_key):
        v = self._tap_dance_counts[td_key] - 1

        if v >= 0:
            if td_key in self._keys_pressed:
                key_to_press = td_key.codes[v]
                self._add_key(key_to_press)
                self._tap_side_effects[td_key] = key_to_press
                self._hid_pending = True
            else:
                if self._tap_side_effects[td_key]:
                    self._remove_key(self._tap_side_effects[td_key])
                    self._tap_side_effects[td_key] = None
                    self._hid_pending = True
                    self._cleanup_tap_dance(td_key)
                else:
                    self._tap_key(td_key.codes[v])
                    self._cleanup_tap_dance(td_key)

        return self

    def _cleanup_tap_dance(self, td_key):
        self._tap_dance_counts[td_key] = 0
        self._tapping = any(count > 0 for count in self._tap_dance_counts.values())
        return self

    def _begin_leader_mode(self):
        if self.leader_mode % 2 == 0:
            self._keys_pressed.discard(KC.LEAD)
            # All leader modes are one number higher when activating
            self.leader_mode += 1

            if self.leader_mode == LeaderMode.TIMEOUT_ACTIVE:
                self._set_timeout(self.leader_timeout, self._handle_leader_sequence)

        return self

    def _handle_leader_sequence(self):
        lmh = tuple(self._leader_mode_history)
        # Will get caught in infinite processing loops if we don't
        # exit leader mode before processing the target key
        self._exit_leader_mode()

        if lmh in self.leader_dictionary:
            # Stack depth exceeded if try to use add_key here with a unicode sequence
            self._process_key(self.leader_dictionary[lmh], True)

            self._set_timeout(
                False, lambda: self._remove_key(self.leader_dictionary[lmh])
            )

        return self

    def _process_leader_mode(self):
        keys_pressed = self._keys_pressed

        if self._leader_last_len and self._leader_mode_history:
            history_set = set(self._leader_mode_history)

            keys_pressed = keys_pressed - history_set

        self._leader_last_len = len(self._keys_pressed)

        for key in keys_pressed:
            if self.leader_mode == LeaderMode.ENTER_ACTIVE and key == KC.ENT:
                self._handle_leader_sequence()
                break
            elif key == KC.ESC or key == KC.GESC:
                # Clean self and turn leader mode off.
                self._exit_leader_mode()
                break
            elif key == KC.LEAD:
                break
            else:
                # Add key if not needing to escape
                # This needs replaced later with a proper debounce
                self._leader_mode_history.append(key)

        self._hid_pending = False
        return self

    def _exit_leader_mode(self):
        self._leader_mode_history.clear()
        self.leader_mode -= 1
        self._leader_last_len = 0
        self._keys_pressed.clear()
        return self

    def _set_timeout(self, after_ticks, callback):
        if after_ticks is False:
            # We allow passing False as an implicit "run this on the next process timeouts cycle"
            timeout_key = ticks_ms()
        else:
            timeout_key = ticks_ms() + after_ticks

        while timeout_key in self._timeouts:
            timeout_key += 1

        self._timeouts[timeout_key] = callback
        return timeout_key

    def _cancel_timeout(self, timeout_key):
        if timeout_key in self._timeouts:
            del self._timeouts[timeout_key]

    def _process_timeouts(self):
        if not self._timeouts:
            return self

        current_time = ticks_ms()

        # cast this to a tuple to ensure that if a callback itself sets
        # timeouts, we do not handle them on the current cycle
        timeouts = tuple(self._timeouts.items())

        for k, v in timeouts:
            if k <= current_time:
                v()
                del self._timeouts[k]

        return self

    #####
    # SPLICE END: INTERNAL STATE
    # TODO FIXME REMOVE THIS
    #####

    def _init_sanity_check(self):
        '''
        Ensure the provided configuration is *probably* bootable
        '''
        assert self.keymap, 'must define a keymap with at least one row'
        assert self.row_pins, 'no GPIO pins defined for matrix rows'
        assert self.col_pins, 'no GPIO pins defined for matrix columns'
        assert self.diode_orientation is not None, 'diode orientation must be defined'
        assert (
            self.hid_type in HIDModes.ALL_MODES
        ), 'hid_type must be a value from kmk.consts.HIDModes'

        return self

    def _init_coord_mapping(self):
        '''
        Attempt to sanely guess a coord_mapping if one is not provided
        '''
        if not self.coord_mapping:
            self.coord_mapping = []

            rows_to_calc = len(self.row_pins)
            cols_to_calc = len(self.col_pins)

            if self.split_offsets:
                rows_to_calc *= 2
                cols_to_calc *= 2

            for ridx in range(rows_to_calc):
                for cidx in range(cols_to_calc):
                    self.coord_mapping.append(intify_coordinate(ridx, cidx))

        return self

    def _init_hid(self):
        if self.hid_type == HIDModes.NOOP:
            self.hid_helper = AbstractHID
        elif self.hid_type == HIDModes.USB:
            self.hid_helper = USBHID
        elif self.hid_type == HIDModes.BLE:
            self.hid_helper = BLEHID

        self._hid_helper_inst = self.hid_helper()

        return self

    def _init_splits(self):
        # Split keyboard Init
        if self.split_type is not None:
            try:
                # Working around https://github.com/adafruit/circuitpython/issues/1769
                self._hid_helper_inst.create_report([]).send()
                self.is_master = True

                # Sleep 2s so master portion doesn't "appear" to boot quicker than
                # dependent portions (which will take ~2s to time out on the HID send)
                sleep_ms(2000)
            except OSError:
                self.is_master = False

            if self.split_flip and not self.is_master:
                self.col_pins = list(reversed(self.col_pins))
            if self.split_side == 'Left':
                self.split_master_left = self.is_master
            elif self.split_side == 'Right':
                self.split_master_left = not self.is_master
        else:
            self.is_master = True

        if self.uart_pin is not None:
            if self.is_master:
                self._uart = busio.UART(
                    tx=None, rx=self.uart_pin, timeout=self.uart_timeout
                )
            else:
                self._uart = busio.UART(
                    tx=self.uart_pin, rx=None, timeout=self.uart_timeout
                )

        gc.collect()

        return self

    def _init_rgb(self):
        if self.rgb_pixel_pin:
            self.pixels = rgb.RGB(self.rgb_config, self.rgb_pixel_pin)
            self.rgb_config = None  # No longer needed
            self.pixels.loopcounter = 0
        else:
            self.pixels = None

        return self

    def _init_led(self):
        if self.led_pin:
            self.led = led.LED(self.led_pin, self.led_config)
            self.led_config = None  # No longer needed
        else:
            self.led = None

        return self

    def _init_matrix(self):
        self.matrix = MatrixScanner(
            cols=self.col_pins,
            rows=self.row_pins,
            diode_orientation=self.diode_orientation,
            rollover_cols_every_rows=getattr(self, 'rollover_cols_every_rows', None),
        )

        return self

    def _init_leader(self):
        '''
        Compile string leader sequences
        '''
        for k, v in self.leader_dictionary.items():
            if not isinstance(k, tuple):
                new_key = tuple(KC[c] for c in k)
                self.leader_dictionary[new_key] = v

        for k, v in self.leader_dictionary.items():
            if not isinstance(k, tuple):
                del self.leader_dictionary[k]

        gc.collect()

        return self

    def go(self, hid_type=HIDModes.USB):
        self._extensions = [] + getattr(self, 'extensions', [])

        try:
            del self.extensions
        except Exception:
            pass
        finally:
            gc.collect()

        self.hid_type = hid_type

        self._init_sanity_check()
        self._init_coord_mapping()
        self._init_hid()

        for ext in self._extensions:
            try:
                ext.during_bootup(self)
            except Exception:
                # TODO FIXME log the exceptions or something
                pass

        self._init_splits()
        self._init_rgb()
        self._init_led()
        self._init_matrix()
        self._init_leader()

        self._print_debug_cycle(init=True)

        while True:
            state_changed = False

            if self.split_type is not None and self.is_master:
                update = self._receive_from_slave()
                if update is not None:
                    self._handle_matrix_report(update)
                    state_changed = True

            for ext in self._extensions:
                try:
                    ext.before_matrix_scan(self)
                except Exception:
                    # TODO FIXME log the exceptions or something
                    pass

            update = self.matrix.scan_for_changes()

            if update is not None:
                if self.is_master:
                    self._handle_matrix_report(update)
                    state_changed = True
                else:
                    # This keyboard is a slave, and needs to send data to master
                    self._send_to_master(update)

            for ext in self._extensions:
                try:
                    ext.before_hid_send(self)
                except Exception:
                    # TODO FIXME log the exceptions or something
                    pass

            if self._hid_pending:
                self._send_hid()

            old_timeouts_len = len(self._timeouts)
            self._process_timeouts()
            new_timeouts_len = len(self._timeouts)

            if old_timeouts_len != new_timeouts_len:
                state_changed = True

                if self._hid_pending:
                    self._send_hid()

            for ext in self._extensions:
                try:
                    ext.after_hid_send(self)
                except Exception:
                    # TODO FIXME log the exceptions or something
                    pass

            if state_changed:
                self._print_debug_cycle()
