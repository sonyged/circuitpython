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
from micropython import const

V0 = const(0)
V1 = const(1)
V2 = const(2)
V3 = const(3)
V4 = const(4)
V5 = const(5)
V6 = const(6)
V7 = const(7)
V8 = const(8)
V9 = const(9)
K0 = const(10)
K1 = const(11)
K2 = const(12)
K3 = const(13)
K4 = const(14)
K5 = const(15)
K6 = const(16)
K7 = const(17)
RGB = const(18)
UP = K2
RIGHT = K3
DOWN = K4
LEFT = K5

PORT2PIN = {
  K2: board.A0,
  K3: board.A1,
  K4: board.A2,
  K5: board.A3,
  K6: board.A4,
  K7: board.A5,
  V2: board.D4,
  V3: board.D3,
  V4: board.D6,
  V5: board.D7,
  V6: board.D8,
  V7: board.D9,
  V8: board.D11,
  V9: board.D13
}

def port2pin(port):
  return PORT2PIN[port]

def clamp(x, y, v):
  return max(x, min(y, v))

DEVICES = []

def finalize():
  while len(DEVICES) > 0:
    DEVICES.pop().deinit()

def execute(main):
  try:
    main()
  finally:
    finalize()

def devices():
  return DEVICES

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
  ON = 'ON'
  OFF = 'OFF'

  def __init__(self, port):
    self._device = digitalio.DigitalInOut(port2pin(port))
    self._device.direction = digitalio.Direction.OUTPUT
    DEVICES.append(self)

  def deinit(self):
    self._device.deinit()

  def on(self):
    self._device.value = True

  def off(self):
    self._device.value = False

  def set_mode(self, mode):
    if mode == led.ON:
      self._device.value = True
    else:
      self._device.value = False

class multi_led:
  def __init__(self, port):
    if port is not RGB:
      raise RuntimeError('port must be koov.RGB, but got', port)
    self._fet = digitalio.DigitalInOut(board.MISO)
    self._fet.direction = digitalio.Direction.OUTPUT
    self._fet.value = True
    self._r = pulseio.PWMOut(board.D13)
    self._g = pulseio.PWMOut(board.D12)
    self._b = pulseio.PWMOut(board.D10)
    DEVICES.append(self)

  def deinit(self):
    self._fet.deinit()
    self._r.deinit()
    self._g.deinit()
    self._b.deinit()

  def on(self, r, g, b):
    self._r.duty_cycle = int((100 - clamp(0, 100, r)) * 0xffff / 100)
    self._g.duty_cycle = int((100 - clamp(0, 100, g)) * 0xffff / 100)
    self._b.duty_cycle = int((100 - clamp(0, 100, b)) * 0xffff / 100)
    self._fet.value = False

  def off(self):
    self._r.duty_cycle = 0xffff
    self._g.duty_cycle = 0xffff
    self._b.duty_cycle = 0xffff
    self._fet.value = True

class buzzer:
  def __init__(self, port):
    self._device = pulseio.PWMOut(
      port2pin(port),
      duty_cycle = 2 ** 15,
      frequency = 20000,
      variable_frequency = True)
    DEVICES.append(self)

  def deinit(self):
    self._device.deinit()

  def on(self, freq):
    def frequency(x): return int(440.0 * 2 ** (((x) - 69) / 12))
    self._device.frequency = frequency(freq)

  def off(self):
    self._device.frequency = 20000

class dc_motor:
  NORMAL = 'NORMAL'
  REVERSE = 'REVERSE'
  COAST = 'COAST'
  BRAKE = 'BRAKE'

  def __init__(self, port, scale = 1):
    if port == V0:
      self._apin = pulseio.PWMOut(board.D2)
      self._dpin = digitalio.DigitalInOut(board.D5)
    elif port == V1:
      self._apin = pulseio.PWMOut(board.D10)
      self._dpin = digitalio.DigitalInOut(board.D12)
    else:
      raise RuntimeError('port must be koov.V0 or koov.V1, but got', port)
    self._dpin.direction = digitalio.Direction.OUTPUT
    self._scale = clamp(0, 1, scale) if scale is not None else 1
    self._mode = dc_motor.COAST
    self._power = 0
    DEVICES.append(self)

  def deinit(self):
    self._apin.deinit()
    self._dpin.deinit()

  def _control(self):
    power = clamp(0, 100, self._power * self._scale)
    if self._mode == dc_motor.NORMAL:
      self._dpin.value = False
      self._apin.duty_cycle = int(power * 0xffff / 100)
    elif self._mode == dc_motor.REVERSE:
      self._dpin.value = True
      self._apin.duty_cycle = int((100 - power) * 0xffff / 100)
    elif self._mode == dc_motor.COAST:
      self._dpin.value = False
      self._apin.duty_cycle = 0
    elif self._mode == dc_motor.BRAKE:
      self._dpin.value = True
      self._apin.duty_cycle = 0xffff
    else:
      raise RuntimeError('unknown mode', self._mode)

  def set_power(self, power):
    self._power = clamp(0, 100, power)
    self._control()

  def set_mode(self, mode):
    self._mode = mode
    self._control()

  def start_normal(self):
    self.set_mode(dc_motor.NORMAL)

  def start_reverse(self):
    self.set_mode(dc_motor.REVERSE)

  def stop_coast(self):
    self.set_mode(dc_motor.COAST)

  def stop_brake(self):
    self.set_mode(dc_motor.BRAKE)

class servo_motor:
  @staticmethod
  def synchronized_motion(speed, servos):
    speed = clamp(0, 100, speed) / 5
    delay = 20 - clamp(0, 20, speed)
    class deltas:
      def __init__(self, servos):
        self._max_delta = 0
        self._curpos = {}
        self._deltas = {}
        for (servo, degree) in servos:
          port = servo.port
          self._curpos[port] = servo.degree
          self._deltas[port] = delta = degree - self._curpos[port]
          self._max_delta = max(self._max_delta, math.fabs(delta))
      @property
      def delta(self):
        return self._max_delta
      def update(self, servo, tick):
        port = servo.port
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

  def __init__(self, port, degree = 90, drift = 0):
    self._port = port
    self._drift = drift if drift is not None else 0
    self._device = pulseio.PWMOut(
      port2pin(self._port), duty_cycle = 2 ** 15, frequency = 50)
    if degree is not None:
      self.set_degree(degree)
    else:
      self._degree = None
    DEVICES.append(self)

  def deinit(self):
    self._device.deinit()

  def set_degree(self, degree):
    self._degree = clamp(0, 180, degree)
    degree = clamp(0, 180, degree + self._drift)
    ratio = degree / 180
    min_duty = (500 * self._device.frequency) / 1000000 * 0xffff
    max_duty = (2500 * self._device.frequency) / 1000000 * 0xffff
    self._device.duty_cycle = int(min_duty + (max_duty - min_duty) * ratio)

  @property
  def degree(self):
    if self._degree is None:
      self.set_degree(90)
    return self._degree

  @property
  def port(self):
    return self._port

class digital_sensor:
  def __init__(self, port):
    self._device = digitalio.DigitalInOut(port2pin(port))
    self._device.switch_to_input(pull = digitalio.Pull.UP)
    DEVICES.append(self)

  def deinit(self):
    self._device.deinit()

  @property
  def value(self):
    return self._device.value

  @property
  def pressed(self):
    return self.value is 0

class push_button(digital_sensor):
  def __init__(self, port):
    super().__init__(port)
core_button = push_button

class touch_sensor(digital_sensor):
  def __init__(self, port):
    super().__init__(port)
push_switch = touch_sensor

class analog_sensor:
  def __init__(self, port, scale = 1.0):
    self._device = analogio.AnalogIn(port2pin(port))
    self._scale = scale
    DEVICES.append(self)

  def deinit(self):
    self._device.deinit()

  @property
  def value(self):
    return self._device.value * self._scale

class ir_photo_reflector(analog_sensor):
  def __init__(self, port):
    super().__init__(port = port, scale = 100.0 / 65535)

class light_sensor(analog_sensor):
  def __init__(self, port):
    super().__init__(port = port, scale = 100.0 / 65535)

class sound_sensor(analog_sensor):
  def __init__(self, port):
    super().__init__(port = port, scale = 100.0 / 65535 * (3.3 - 1.5) / 3.3)

def ensure_i2c(port):
  if port not in [ K0, K1 ]:
    raise RuntimeError('port must be koov.K0 or koov.K1, but got', port)

class accelerometer:
  def __init__(self, port):
    ensure_i2c(port)
    self._i2c = busio.I2C(board.SCL, board.SDA)
    self._device = adafruit_mma8451.MMA8451(self._i2c)
    DEVICES.append(self)

  def deinit(self):
    self._i2c.deinit()

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

class image:
  GRB = 'grb'

  def __init__(self, height, width, format, pixels):
    raise RuntimeError('image is not implemented')

class led_matrix:
  def __init__(self, port):
    raise RuntimeError('led-matrix is not implemented')

  def show(self, image, x, y, brightness):
    raise RuntimeError('led-matrix is not implemented')
multicolor_matrix = led_matrix

class ultrasonic_distance_sensor:
  def __init__(self, port):
    raise RuntimeError('ultrasonic distance sensor is not implemented')

  @property
  def value(self):
    raise RuntimeError('ultrasonic distance sensor is not implemented')

class color_sensor:
  def __init__(self, port):
    ensure_i2c(port)
    raise RuntimeError('color sensor is not implemented')

  def deinit(self):
    raise RuntimeError('color sensor is not implemented')

  @property
  def value(self):
    raise RuntimeError('color sensor is not implemented')

  @property
  def r(self):
    raise RuntimeError('color sensor is not implemented')

  @property
  def g(self):
    raise RuntimeError('color sensor is not implemented')

  @property
  def b(self):
    raise RuntimeError('color sensor is not implemented')
