"""QEMU ZedMon client."""

import zmq
import re
import logging
import time
from .util import StoppableThread
from collections import deque


class ZEvtClient(StoppableThread):
    """Client."""

    _GPIO_EVT_REGEX = re.compile(
        r'GPIO\s+([0-9]+)\s+CHANNEL\s+([0-9]+)\s+EVENT\s+([a-zA-Z0-9]+)\s+'
        r'DATA\s+([a-zA-Z0-9]+)\s+VALUE\s+([a-zA-Z0-9]+).*')
    _TIMER_EVT_REGEX = re.compile(r'TIMER\s+EVENT\s+([a-zA-Z0-9]+)\s+'
        r'DATA\s+([a-zA-Z0-9]+).*')

    def __init__(self, address, port, evt_callback=None):
        """Initialize."""
        super().__init__()
        self._srv_address = 'tcp://{}:{}'.format(address, port)
        self._evt_cb = evt_callback
        self._logger = logging.getLogger('zpanel.zcli')

    def run(self):
        """Run main loop."""
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(self._srv_address)
        socket.setsockopt_string(zmq.SUBSCRIBE, 'GPIO')
        socket.setsockopt_string(zmq.SUBSCRIBE, 'TIMER')
        self._logger.debug('event client started')

        while True:
            if self.is_stopped():
                # quit thread
                exit(0)

            try:
                evt_str = socket.recv_string(flags=zmq.NOBLOCK)
                #self._logger.debug('got string: ', evt_str)
                m = self._GPIO_EVT_REGEX.match(evt_str)
                if m is not None:
                    # gpio event
                    if self._evt_cb is not None:
                        self._evt_cb(
                            {'type': 'gpio',
                             'chip': int(m.group(1)),
                             'channel': int(m.group(2)),
                             'event': m.group(3),
                             'data': int(m.group(4)),
                             'value': int(m.group(5))})  #int(m.group(5).split('0x')[1], 16)})
                m = self._TIMER_EVT_REGEX.match(evt_str)
                if m is not None:
                    if self._evt_cb is not None:
                        self._evt_cb(
                            {'type': 'timer',
                             'event': m.group(1),
                             'data': int(m.group(2).split('0x')[1], 16)})

            except zmq.ZMQError:
                pass

            time.sleep(0.01)


class ZCmdClient(StoppableThread):
    """Client."""

    _PERIPHERAL_DESCR_REGEX = re.compile(r'([0-9]+)\s+([0-9a-zA-Z]+)\s+'
                                         r'([0-9a-zA-Z]+)\s*(.*)')
    _PERIPHERAL_LIST_CMD = 0x00
    _GPIO_GET_STATE = 0x01
    _GPIO_GET_DIR = 0x02
    _GPIO_SET_DIR = 0x03

    def __init__(self, address, port):
        """Initialize."""
        super().__init__()
        self._srv_address = 'tcp://{}:{}'.format(address, port)
        self._logger = logging.getLogger('zpanel.zcmd')
        self._requests = deque()

    def _decode_peripheral_list(self, response):
        peripherals = response.split('\n')
        peripheral_dict = {}
        for peripheral in peripherals:
            m = self._PERIPHERAL_DESCR_REGEX.match(peripheral.strip())
            if m is not None:
                peripheral_dict[int(m.group(1))] = {
                    'type': m.group(2),
                    'label': m.group(3)
                }

        return peripheral_dict

    def _queue_command(self, command, args=None, callback=None):
        self._requests.appendleft({'command': command,
                                   'args': args,
                                   'callback': callback})

    def _send_command(self, socket, command, args=None, callback=None):
        if command == self._PERIPHERAL_LIST_CMD:
            socket.send_string('PERIPHERALS')

            # receive response
            try:
                resp = socket.recv_string()
                if callback is not None:
                    callback(self._decode_peripheral_list(resp))
            except zmq.ZMQError:
                self._logger.debug('error receiving response')
        elif command == self._GPIO_GET_STATE:
            socket.send_string('GPIO GET VALUE {}'.format(args[0]))

            try:
                resp = socket.recv_string().strip()
                if callback is not None:
                    callback(int(resp.split('0x')[1], 16))
            except zmq.ZMQError:
                self._logger.debug('error receiving response')
        elif command == self._GPIO_GET_DIR:
            socket.send_string('GPIO GET DIRECTION {}'.format(args[0]))

            try:
                resp = socket.recv_string().strip()
                if callback is not None:
                    callback(int(resp.split('0x')[1], 16))
            except zmq.ZMQError:
                self._logger.debug('error receiving response')
        elif command == self._GPIO_SET_DIR:
            socket.send_string('GPIO SET {} {}'.format(args[0], args[1]))

            try:
                resp = socket.recv_string().strip()
            except zmq.ZMQError:
                self._logger.debug('error receiving response')

            # no response

    def get_peripherals(self, callback=None):
        """Get peripheral list."""
        self._queue_command(self._PERIPHERAL_LIST_CMD, callback=callback)

    def get_gpio_state(self, gpiochip, callback=None):
        """Get GPIO state."""
        self._queue_command(self._GPIO_GET_STATE, [gpiochip],
                            callback=callback)

    def get_gpio_dir(self, gpiochip, callback=None):
        """Get GPIO state."""
        self._queue_command(self._GPIO_GET_DIR, [gpiochip],
                            callback=callback)

    def set_gpio_state(self, gpiochip, value):
        """Set values."""
        self._queue_command(self._GPIO_SET_DIR, [gpiochip, value])

    def run(self):
        """Run loop."""
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(self._srv_address)
        self._logger.debug('cmd client started')

        while True:
            if self.is_stopped():
                exit(0)

            if len(self._requests) > 0:
                # process requests
                req = self._requests.pop()
                self._send_command(socket, **req)

            time.sleep(0.01)
