# koov support module.

import board
import digitalio
import analogio
import pulseio
import time
import random
import math
import busio
import adafruit_mma8451

TIMER_EPOCH = time.monotonic()

def reset_timer():
  global TIMER_EPOCH
  TIMER_EPOCH = time.monotonic()

def timer():
  return time.monotonic() - TIMER_EPOCH

def wait(secs):
  time.sleep(max(0, secs))

def pick_random(x, y):
  return random.randint(math.trunc(x), math.trunc(y))

class led:
  def __init__(self, port):
    self._device = digitalio.DigitalInOut(port)
    self._device.direction = digitalio.Direction.OUTPUT

  def on(self):
    self._device.value = True

  def off(self):
    self._device.value = False

  def set_mode(self, mode):
    if mode == 'ON':
      self._device.value = True
    else:
      self._device.value = False

class multi_led:
  def __init__(self):
    self._fet = digitalio.DigitalInOut(board.MISO)
    self._fet.direction = digitalio.Direction.OUTPUT
    self._fet.value = True
    self._r = pulseio.PWMOut(board.D13)
    self._g = pulseio.PWMOut(board.D12)
    self._b = pulseio.PWMOut(board.D10)

  def on(self, r, g, b):
    self._r.duty_cycle = int((100 - max(0, min(100, r))) * 0xffff / 100)
    self._g.duty_cycle = int((100 - max(0, min(100, g))) * 0xffff / 100)
    self._b.duty_cycle = int((100 - max(0, min(100, b))) * 0xffff / 100)
    self._fet.value = False

  def off(self):
    self._r.duty_cycle = 0xffff
    self._g.duty_cycle = 0xffff
    self._b.duty_cycle = 0xffff
    self._fet.value = True

class buzzer:
  def __init__(self, port):
    self._device = pulseio.PWMOut(port,
                                  duty_cycle = 2 ** 15,
                                  frequency = 20000,
                                  variable_frequency = True)

  def on(self, freq):
    def frequency(x): return int(440.0 * 2 ** (((x) - 69) / 12))
    self._device.frequency = frequency(freq)

  def off(self):
    self._device.frequency = 20000

class dc_motor:
  def __init__(self, port):
    if port == board.V0:
      self._apin = pulseio.PWMOut(board.D2)
      self._dpin = digitalio.DigitalInOut(board.D5)
    elif port == board.V1:
      self._apin = pulseio.PWMOut(board.D10)
      self._dpin = digitalio.DigitalInOut(board.D12)
    else:
      raise RuntimeError('port must be board.V0 or board.V1, but got', port)
    self._dpin.direction = digitalio.Direction.OUTPUT
    self._mode = 'COAST'
    self._power = 0

  def _control(self):
    if self._mode == 'NORMAL':
      self._dpin.value = False
      self._apin.duty_cycle = int(self._power * 0xffff / 100)
    elif self._mode == 'REVERSE':
      self._dpin.value = True
      self._apin.duty_cycle = int((100 - self._power) * 0xffff / 100)
    elif self._mode == 'COAST':
      self._dpin.value = False
      self._apin.duty_cycle = 0
    elif self._mode == 'BRAKE':
      self._dpin.value = True
      self._apin.duty_cycle = 0xffff
    else:
      raise RuntimeError('unknown mode', self._mode)

  def set_power(self, power):
    self._power = max(0, min(100, power))
    self._control()

  def set_mode(self, mode):
    self._mode = mode
    self._control()

class servo_motor:
  @staticmethod
  def synchronized_motion(speed, servos):
    speed = max(0, min(100, speed)) / 5
    delay = 20 - max(0, min(20, speed))
    class deltas:
      def __init__(self, servos):
        self._max_delta = 0
        self._curpos = {}
        self._deltas = {}
        for (servo, degree) in servos:
          port = self.portname(servo.port)
          self._curpos[port] = servo.degree
          self._deltas[port] = delta = degree - self._curpos[port]
          self._max_delta = max(self._max_delta, math.fabs(delta))
      @staticmethod
      def portname(port):
        if port == board.V2:
          return 'V2'
        elif port == board.V3:
          return 'V3'
        elif port == board.V4:
          return 'V4'
        elif port == board.V5:
          return 'V5'
        elif port == board.V6:
          return 'V6'
        elif port == board.V7:
          return 'V7'
        elif port == board.V8:
          return 'V8'
        elif port == board.V9:
          return 'V9'
        else:
          raise RuntimeError('port must be V2 through board.V9, but got', port)
      @property
      def delta(self):
        return self._max_delta
      def update(self, servo, tick):
        port = self.portname(servo.port)
        return self._curpos[port] + self._deltas[port] / self._max_delta * tick
    deltas = deltas(servos)
    if deltas.delta == 0:
      pass
    else:
      def moveall(f, delay):
        for (servo, degree) in servos: f(servo, degree)
        time.sleep(delay / 1000)
      if delay == 0:
        def set_degree(servo, degree): servo.set_degree(degree = degree)
        moveall(set_degree, deltas.delta * 3)
      else:
        for tick in range(0, deltas.delta):
          def move_1tick(servo, degree):
            servo.set_degree(degree = deltas.update(servo, tick))
          moveall(move_1tick, delay)

  def __init__(self, port, degree = None):
    self._port = port
    self._device = pulseio.PWMOut(port, duty_cycle = 2 ** 15, frequency = 50)
    if degree:
      self.set_degree(degree)
    else:
      self._degree = None

  def set_degree(self, degree):
    degree = max(0, min(180, degree))
    ratio = degree / 180
    self._degree = degree
    min_duty = (500 * self._device.frequency) / 1000000 * 0xffff
    max_duty = (2500 * self._device.frequency) / 1000000 * 0xffff
    self._device.duty_cycle = int(min_duty + (max_duty - min_duty) * ratio)

  @property
  def degree(self):
    if not self._degree:
      self.set_degree(90)
    return self._degree

  @property
  def port(self):
    return self._port

class digital_sensor:
  def __init__(self, port):
    self._device = digitalio.DigitalInOut(port)
    self._device.switch_to_input(pull = digitalio.Pull.UP)
  @property
  def value(self):
    return self._device.value

class push_button(digital_sensor):
  def __init__(self, port):
    super().__init__(port)

class touch_sensor(digital_sensor):
  def __init__(self, port):
    super().__init__(port)

class analog_sensor:
  def __init__(self, port, scale = 1.0):
    self._device = analogio.AnalogIn(port)
    self._scale = scale
  @property
  def value(self):
    return self._device.value * self._scale

class ir_photo_reflector(analog_sensor):
  def __init__(self, port):
    super().__init__(port = port, scale = 100.0 / 65535)

class light_sensor(analog_sensor):
  def __init__(self, port):
    super().__init__(port = port, scale = 100.0 / 65535)

class accelerometer:
  def __init__(self):
    self._i2c = busio.I2C(board.SCL, board.SDA)
    self._device = adafruit_mma8451.MMA8451(self._i2c)

  @property
  def value(self):
    return self._device.acceleration

  @property
  def x(self):
    x, y, z = self._device.acceleration
    return x

  @property
  def y(self):
    x, y, z = self._device.acceleration
    return y

  @property
  def z(self):
    x, y, z = self._device.acceleration
    return z
