"""Web interface."""

import logging
import pkg_resources
from bottle import (route, run, template, static_file, request, ServerAdapter,
                    TEMPLATE_PATH)


class ZWebInterface:
    """Web interface class."""

    _LED_GPIOCHIP = 3
    _PUSHBTN_GPIOCHIP = 1
    _DIPSW_GPIOCHIP = 2
    _PMOD_GPIOCHIP = 4

    def __init__(self, bind_addr, port, zmqcli):
        """Initialize."""
        self._bind_to = bind_addr
        self._srv_port = port
        self._logger = logging.getLogger('zpanel.zweb')
        self._cli = zmqcli

        # get resources
        try:
            views = pkg_resources.resource_filename('zmon',
                                                    'data/views')

            TEMPLATE_PATH.insert(0, views)

            self.static_file_path =\
                pkg_resources.resource_filename('zmon',
                                                'data/web_static')
        except pkg_resources.DistributionNotFound:
            raise FileNotFoundError('cant find resources')

        # store state
        self._led_dir_state = 0x00
        self._led_value = 0
        self._dip_state = 0x00
        self._led_duty_cycle = 0

    def jsFiles(self, filename):
        """Retrieve .js files."""
        return static_file(filename, root=self.static_file_path)

    def cssFiles(self, filename):
        """Retrieve css files."""
        return static_file(filename, root=self.static_file_path)

    def imgFiles(self, filename):
        """Retrieve image files."""
        return static_file(filename, root=self.static_file_path)

    def fontFiles(self, filename):
        """Retrieve font files."""
        return static_file(filename, root='data/web_static/fonts')

    def register_event(self, evt_desc):
        """Event callback."""

        if evt_desc['type'] == 'gpio':
            if evt_desc['chip'] == self._LED_GPIOCHIP:
                if evt_desc['channel'] == 0:
                    if evt_desc['event'] == 'DIR':
                        self._led_dir_state = evt_desc['data']
                    elif evt_desc['event'] == 'VALUE':
                        self._led_value = evt_desc['value']
                        self._led_duty_cycle = evt_desc['data']
                        self._logger.debug('setting LED {} state: {}'
                                           .format(self._led_value, self._led_duty_cycle))
        if evt_desc['type'] == 'timer':
            self._logger.debug(evt_desc)
            if evt_desc['event'] == 'DUTY':
                self._led_state = evt_desc['data']

    def index(self):
        """Return main and only page."""
        return template('index')

    def _recv_peripherals(self, plist):
        # print(plist)
        pass

    def _recv_led_state(self, value):
        self._led_value = value

    def _recv_led_dir(self, value):
        self._led_dir_state = value

    def _get_leds_state(self):
        # if direction is set to input then turn off
        led_state = {}
        led_state[0] = self._led_state
        return {'status': 'ok',
                'data': led_state}

    def _set_sw_state(self):
        state = request.POST

        if 'swn' not in state:
            return {'status': 'error'}

        if 'value' not in state:
            return {'status': 'error'}

        swn = state['swn']
        if int(swn) < 0 or int(swn) > 7:
            return {'status': 'error'}

        if bool(int(state['value'])):
            self._dip_state |= (1 << int(swn))
        else:
            self._dip_state &= ~(1 << int(swn))

        self._cli.set_gpio_state(self._DIPSW_GPIOCHIP, self._dip_state)

    def run(self):
        """Run server."""
        route('/')(self.index)
        route("/js/<filename:re:.*\.js>")(self.jsFiles)
        route("/css/<filename:re:.*\.css>")(self.cssFiles)
        route("/images/<filename:re:.*\.(jpg|png|gif|ico)>")(self.imgFiles)
        route("/fonts/<filename:re:.*\.(eot|ttf|woff|svg)>")(self.fontFiles)

        # status getters and setters
        route('/state/leds')(self._get_leds_state)
        route('/state/sw', method='POST')(self._set_sw_state)

        # get peripheral list
        self._cli.get_peripherals(self._recv_peripherals)

        # reset switches
        self._cli.set_gpio_state(self._DIPSW_GPIOCHIP, 0x00)

        # get current led state
        self._cli.get_gpio_state(self._LED_GPIOCHIP, self._recv_led_state)
        self._cli.get_gpio_dir(self._LED_GPIOCHIP, self._recv_led_dir)

        self._logger.debug('starting web interface')
        run(server='cherrypy', host=self._bind_to, port=self._srv_port)
