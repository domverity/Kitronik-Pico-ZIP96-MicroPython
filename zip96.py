"""
zip96 - an improved MicroPython driver for the Kitronik ZIP96 Retro Gamer
(product code 5347) for the Raspberry Pi Pico.

Improvements over the stock ZIP96Pico.py:

  * Buttons are interrupt driven (via picozero) with debouncing - attach
    handlers with ``gamer.a.when_pressed = my_function`` instead of polling.
  * The LED matrix driver keeps the PIO/WS2812 state machine but renders
    through a pre-computed brightness lookup table into a pre-allocated
    buffer, so ``show()`` does no floating point maths and no allocation.
  * Optional gamma correction for perceptually even brightness.
  * Drawing primitives: pixels, lines, rectangles and string-art sprites.
  * A built-in 3x5 font with static and scrolling text rendering.
  * A Menu class driven by the D-pad with A to select and B to cancel.
  * The buzzer is a picozero Speaker, so it can play notes by name and
    whole tunes, blocking or in the background.
  * The vibration motor supports timed, non-blocking pulses.

Requires picozero.py alongside this file on the Pico.

Quick start::

    from zip96 import ZIP96, RED, WHITE

    gamer = ZIP96(brightness=15)
    gamer.a.when_pressed = lambda: gamer.vibration.buzz(0.1)
    gamer.screen.scroll_text("HELLO WORLD", RED)
    choice = gamer.menu(["SNAKE", "TETRIS", "OFF"])

The original Kitronik API (KitronikZIP96, Screen.setLEDMatrix, Buzzer.playTone,
Vibrate.vibrate, button.pressed() etc.) is kept as aliases, so existing games
run after changing ``import ZIP96Pico`` to ``import zip96 as ZIP96Pico``.
"""

import micropython
from array import array
from machine import Pin
from micropython import const
from rp2 import PIO, StateMachine, asm_pio
from time import sleep_ms, ticks_ms, ticks_diff

from picozero import Button as _Button, DigitalOutputDevice, Speaker

# Pin assignments from the Kitronik 5347 datasheet
UP_PIN = const(14)
DOWN_PIN = const(12)
LEFT_PIN = const(13)
RIGHT_PIN = const(15)
A_PIN = const(1)
B_PIN = const(2)
BUZZER_PIN = const(5)
VIBRATE_PIN = const(4)
ZIP_PIN = const(7)

WIDTH = const(12)
HEIGHT = const(8)
NUM_LEDS = const(96)

BLACK = (0, 0, 0)
RED = (255, 0, 0)
ORANGE = (255, 100, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
MAGENTA = (255, 0, 100)
WHITE = (255, 255, 255)
GREY = (40, 40, 40)
COLOURS = (BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)

# 3x5 font, ASCII 32..95, five rows per glyph, bit 2 = leftmost pixel.
# Lowercase letters are folded to uppercase; anything else renders as '?'.
_FONT_START = const(32)
_FONT_END = const(95)
_FONT = bytes((
    0, 0, 0, 0, 0,  2, 2, 2, 0, 2,  5, 5, 0, 0, 0,  5, 7, 5, 7, 5,  #  !"#
    3, 6, 2, 3, 6,  5, 1, 2, 4, 5,  2, 5, 2, 5, 3,  2, 2, 0, 0, 0,  # $%&'
    2, 4, 4, 4, 2,  2, 1, 1, 1, 2,  0, 5, 2, 5, 0,  0, 2, 7, 2, 0,  # ()*+
    0, 0, 0, 2, 4,  0, 0, 7, 0, 0,  0, 0, 0, 0, 2,  1, 1, 2, 4, 4,  # ,-./
    7, 5, 5, 5, 7,  2, 6, 2, 2, 7,  7, 1, 7, 4, 7,  7, 1, 3, 1, 7,  # 0123
    5, 5, 7, 1, 1,  7, 4, 7, 1, 7,  7, 4, 7, 5, 7,  7, 1, 1, 2, 2,  # 4567
    7, 5, 7, 5, 7,  7, 5, 7, 1, 7,  0, 2, 0, 2, 0,  0, 2, 0, 2, 4,  # 89:;
    1, 2, 4, 2, 1,  0, 7, 0, 7, 0,  4, 2, 1, 2, 4,  7, 1, 2, 0, 2,  # <=>?
    7, 5, 7, 4, 7,  2, 5, 7, 5, 5,  6, 5, 6, 5, 6,  3, 4, 4, 4, 3,  # @ABC
    6, 5, 5, 5, 6,  7, 4, 6, 4, 7,  7, 4, 6, 4, 4,  3, 4, 5, 5, 3,  # DEFG
    5, 5, 7, 5, 5,  7, 2, 2, 2, 7,  1, 1, 1, 5, 2,  5, 6, 4, 6, 5,  # HIJK
    4, 4, 4, 4, 7,  5, 7, 5, 5, 5,  6, 5, 5, 5, 5,  2, 5, 5, 5, 2,  # LMNO
    6, 5, 6, 4, 4,  2, 5, 5, 2, 1,  6, 5, 6, 5, 5,  3, 4, 2, 1, 6,  # PQRS
    7, 2, 2, 2, 2,  5, 5, 5, 5, 7,  5, 5, 5, 5, 2,  5, 5, 5, 7, 5,  # TUVW
    5, 5, 2, 5, 5,  5, 5, 2, 2, 2,  7, 1, 2, 4, 7,  3, 2, 2, 2, 3,  # XYZ[
    4, 4, 2, 1, 1,  6, 2, 2, 2, 6,  2, 5, 0, 0, 0,  0, 0, 0, 0, 7,  # \]^_
))

_used_state_machines = [False] * 8


def _claim_state_machine(program, freq, sideset_base):
    for i in range(8):
        if _used_state_machines[i]:
            continue
        try:
            sm = StateMachine(i, program, freq=freq, sideset_base=sideset_base)
        except ValueError:
            continue  # claimed by something outside this module
        _used_state_machines[i] = True
        return sm
    raise RuntimeError("No free PIO state machine for the ZIP LEDs")


class Button(_Button):
    """One of the six gamer buttons. Active high (the board pulls the pin
    down), interrupt driven and debounced by picozero.

    Use ``is_pressed`` for polling and ``when_pressed`` / ``when_released``
    for event handlers.
    """

    def __init__(self, pin, name=""):
        super().__init__(pin, pull_up=False)
        self.name = name

    # Original Kitronik API
    def pressed(self):
        return bool(self.value)


class Vibration(DigitalOutputDevice):
    """The vibration motor. ``buzz(seconds)`` pulses without blocking;
    ``on()``/``off()``/``blink()`` come from picozero.
    """

    def __init__(self, pin=VIBRATE_PIN):
        super().__init__(pin)

    def buzz(self, seconds=0.2):
        self.on(t=seconds, wait=False)

    # Original Kitronik API
    def vibrate(self):
        self.on()

    def stop(self):
        self.off()


class Buzzer(Speaker):
    """The piezo buzzer as a picozero Speaker, so as well as plain tones it
    plays note names and tunes, e.g.::

        buzzer.play("c4", 0.5)
        buzzer.play([("e4", 0.2), ("g4", 0.2), ("c5", 0.4)], wait=False)
    """

    def __init__(self, pin=BUZZER_PIN):
        # 50% duty cycle (32767) is the loudest square wave for a piezo
        super().__init__(pin, duty_factor=32767)

    def play_tone(self, freq, duration=None, wait=True):
        """Play a tone in Hz (clamped to the piezo's 30-3000Hz range).
        With no duration it sounds until ``stop()`` is called.
        """
        freq = min(3000, max(30, freq))
        if duration is None:
            self.value = (freq, 1)
        else:
            self.play(freq, duration, wait=wait)

    def stop(self):
        self.off()

    # Original Kitronik API
    def playTone(self, freq):
        self.play_tone(freq)

    def playTone_Length(self, freq, length):
        self.play_tone(freq, length / 1000)

    def stopTone(self):
        self.off()


class Screen:
    """The 12x8 ZIP LED matrix.

    Colours are (r, g, b) tuples, 0-255 per channel. Drawing calls update an
    off-screen buffer; call ``show()`` to push it to the LEDs. Coordinates
    off the edge of the matrix are clipped silently, so sprites and text can
    move partly (or wholly) off screen.
    """

    width = WIDTH
    height = HEIGHT

    # Colour attributes kept from the original Kitronik API
    BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE = (
        BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)
    COLOURS = COLOURS

    # WS2812 output: 8MHz state machine clock, 10 cycles per bit = 1.25us
    @asm_pio(sideset_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_LEFT,
             autopull=True, pull_thresh=24)
    def _pio_program():
        T1 = 2
        T2 = 5
        T3 = 3
        wrap_target()
        label("bitloop")
        out(x, 1)               .side(0)    [T3 - 1]
        jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
        jmp("bitloop")          .side(1)    [T2 - 1]
        label("do_zero")
        nop()                   .side(0)    [T2 - 1]
        wrap()

    def __init__(self, pin=ZIP_PIN, brightness=20, gamma=1.0):
        self._sm = _claim_state_machine(self._pio_program, 8_000_000, Pin(pin))
        self._buf = array("I", (0 for _ in range(NUM_LEDS)))   # packed GRB
        self._out = array("I", (0 for _ in range(NUM_LEDS)))   # brightness applied
        self._lut = bytearray(256)
        self._gamma = gamma
        self._brightness = 0
        self.brightness = brightness
        self._sm.active(1)

    @property
    def brightness(self):
        """Brightness 0-100, applied in ``show()``."""
        return self._brightness

    @brightness.setter
    def brightness(self, value):
        value = min(100, max(0, value))
        if value == self._brightness:
            return
        self._brightness = value
        scale = value / 100
        lut = self._lut
        if self._gamma == 1.0:
            for i in range(256):
                lut[i] = int(i * scale + 0.5)
        else:
            for i in range(256):
                lut[i] = int(255 * (i / 255) ** self._gamma * scale + 0.5)

    @micropython.native
    def show(self):
        """Push the drawing buffer to the LEDs."""
        lut = self._lut
        buf = self._buf
        out = self._out
        for i in range(NUM_LEDS):
            c = buf[i]
            out[i] = (lut[(c >> 16) & 0xFF] << 16) | (lut[(c >> 8) & 0xFF] << 8) | lut[c & 0xFF]
        self._sm.put(out, 8)

    @staticmethod
    def _pack(colour):
        return (colour[1] << 16) | (colour[0] << 8) | colour[2]

    def set_pixel(self, x, y, colour):
        """Set the pixel at (x, y). Off-screen coordinates are ignored."""
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            self._buf[x + y * WIDTH] = self._pack(colour)

    def get_pixel(self, x, y):
        """Return the buffered (r, g, b) at (x, y) - black if off screen."""
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            c = self._buf[x + y * WIDTH]
            return ((c >> 8) & 0xFF, (c >> 16) & 0xFF, c & 0xFF)
        return BLACK

    def set_led(self, index, colour):
        """Set an LED by strip index 0-95 (raises IndexError if out of range)."""
        if not 0 <= index < NUM_LEDS:
            raise IndexError("LED index out of range: %d" % index)
        self._buf[index] = self._pack(colour)

    def get_led(self, index):
        if not 0 <= index < NUM_LEDS:
            raise IndexError("LED index out of range: %d" % index)
        c = self._buf[index]
        return ((c >> 8) & 0xFF, (c >> 16) & 0xFF, c & 0xFF)

    def fill(self, colour):
        packed = self._pack(colour)
        buf = self._buf
        for i in range(NUM_LEDS):
            buf[i] = packed

    def clear(self, show=False):
        """Blank the buffer; pass show=True to also blank the LEDs."""
        self.fill(BLACK)
        if show:
            self.show()

    def hline(self, x, y, length, colour):
        for i in range(length):
            self.set_pixel(x + i, y, colour)

    def vline(self, x, y, length, colour):
        for i in range(length):
            self.set_pixel(x, y + i, colour)

    def rect(self, x, y, w, h, colour, fill=False):
        if fill:
            for row in range(h):
                self.hline(x, y + row, w, colour)
        else:
            self.hline(x, y, w, colour)
            self.hline(x, y + h - 1, w, colour)
            self.vline(x, y + 1, h - 2, colour)
            self.vline(x + w - 1, y + 1, h - 2, colour)

    def blit(self, rows, x=0, y=0, palette=None):
        """Draw a string-art sprite with its top-left corner at (x, y).

        ``rows`` is a list of equal-ish length strings; ``palette`` maps each
        character to a colour. Spaces and '.' are transparent, characters not
        in the palette draw white::

            heart = ["r.r", "rrr", ".r."]
            screen.blit(heart, 4, 2, {"r": RED})
        """
        palette = palette or {}
        for dy, row in enumerate(rows):
            for dx, ch in enumerate(row):
                if ch == " " or ch == ".":
                    continue
                self.set_pixel(x + dx, y + dy, palette.get(ch, WHITE))

    # ------------------------------------------------------------- text ---

    @staticmethod
    def text_width(text):
        """Pixel width of ``text`` in the built-in font (4px per character)."""
        return max(0, len(str(text)) * 4 - 1)

    def _draw_glyph(self, char, x, y, packed):
        code = ord(char)
        if 97 <= code <= 122:       # fold lowercase to uppercase
            code -= 32
        if not _FONT_START <= code <= _FONT_END:
            code = 63               # '?'
        base = (code - _FONT_START) * 5
        buf = self._buf
        for row in range(5):
            bits = _FONT[base + row]
            yy = y + row
            if bits and 0 <= yy < HEIGHT:
                if bits & 4 and 0 <= x < WIDTH:
                    buf[x + yy * WIDTH] = packed
                if bits & 2 and 0 <= x + 1 < WIDTH:
                    buf[x + 1 + yy * WIDTH] = packed
                if bits & 1 and 0 <= x + 2 < WIDTH:
                    buf[x + 2 + yy * WIDTH] = packed

    def draw_text(self, text, x=0, y=1, colour=WHITE):
        """Draw ``text`` with its top-left corner at (x, y). Characters are
        3x5 pixels plus a 1px gap; anything off screen is clipped, so x may
        be negative (useful for scrolling). Returns the x position after the
        last character.
        """
        packed = self._pack(colour)
        for char in str(text):
            if x >= WIDTH:
                break
            if x > -4:
                self._draw_glyph(char, x, y, packed)
            x += 4
        return x

    def centre_text(self, text, y=1, colour=WHITE):
        """Draw short text horizontally centred (3 characters fit)."""
        self.draw_text(text, (WIDTH - self.text_width(text)) // 2, y, colour)

    def scroll_text(self, text, colour=WHITE, y=1, background=BLACK,
                    step_ms=80, stop=None):
        """Scroll ``text`` across the screen once (blocking).

        ``stop`` is an optional callable checked before each frame - return
        True to abort, e.g. ``stop=gamer.b.pressed``.
        """
        text = str(text)
        end = -(self.text_width(text) + 1)
        for offset in range(WIDTH, end, -1):
            if stop and stop():
                break
            self.fill(background)
            self.draw_text(text, offset, y, colour)
            self.show()
            sleep_ms(step_ms)

    # Original Kitronik API
    def setLED(self, whichLED, whichColour):
        self.set_led(whichLED, whichColour)

    def getLED(self, whichLED):
        return self.get_led(whichLED)

    def setLEDMatrix(self, X, Y, whichColour):
        self.set_pixel(X, Y, whichColour)

    def setBrightness(self, value):
        self.brightness = value


class Menu:
    """A scrolling text menu on the LED matrix.

    Up/Down move through the items (labels scroll if they are longer than
    three characters), A selects, B cancels. A row of dots along the bottom
    shows the current position. ``run()`` blocks and returns the selected
    index, or None if cancelled::

        menu = Menu(gamer, ["SNAKE", "PONG", "OFF"])
        choice = menu.run()
    """

    def __init__(self, gamer, items, colour=WHITE, highlight=CYAN,
                 background=BLACK, indicator=GREY, step_ms=120, sounds=True):
        self._gamer = gamer
        self._items = [str(item) for item in items]
        self._colour = colour
        self._highlight = highlight
        self._background = background
        self._indicator = indicator
        self._step_ms = step_ms
        self._sounds = sounds
        self._nav = 0
        self._action = None

    def _select_item(self, step):
        self._nav += step

    def _click(self, freq):
        if self._sounds and self._gamer.buzzer:
            self._gamer.buzzer.play(freq, 0.04, wait=False)

    def run(self):
        if not self._items:
            return None
        gamer = self._gamer
        screen = gamer.screen
        buttons = (gamer.up, gamer.down, gamer.a, gamer.b)
        saved_handlers = [b.when_pressed for b in buttons]
        gamer.up.when_pressed = lambda: self._select_item(-1)
        gamer.down.when_pressed = lambda: self._select_item(1)
        gamer.a.when_pressed = lambda: setattr(self, "_action", "select")
        gamer.b.when_pressed = lambda: setattr(self, "_action", "cancel")
        self._nav = 0
        self._action = None
        index = 0
        tick = 0
        last_step = ticks_ms()
        try:
            while True:
                if self._nav:
                    index = (index + self._nav) % len(self._items)
                    self._nav = 0
                    tick = 0
                    self._click(660)
                if self._action == "select":
                    self._click(990)
                    if gamer.vibration:
                        gamer.vibration.buzz(0.08)
                    return index
                if self._action == "cancel":
                    self._click(220)
                    return None
                self._draw(index, tick)
                sleep_ms(15)
                if ticks_diff(ticks_ms(), last_step) >= self._step_ms:
                    last_step = ticks_ms()
                    tick += 1
        finally:
            for button, handler in zip(buttons, saved_handlers):
                button.when_pressed = handler
            screen.clear(show=True)

    def _draw(self, index, tick):
        screen = self._gamer.screen
        screen.fill(self._background)
        label = self._items[index]
        width = screen.text_width(label)
        if width <= WIDTH:
            screen.draw_text(label, (WIDTH - width) // 2, 1, self._colour)
        else:
            # pause at each end of the scroll, slide in between
            span = width - WIDTH
            pause = 8
            phase = tick % (span + 2 * pause)
            offset = min(max(phase - pause, 0), span)
            screen.draw_text(label, -offset, 1, self._colour)
        count = len(self._items)
        if self._indicator and count > 1 and count <= WIDTH:
            start = (WIDTH - count) // 2
            for i in range(count):
                screen.set_pixel(start + i, HEIGHT - 1,
                                 self._highlight if i == index else self._indicator)
        screen.show()


class ZIP96:
    """The whole handheld: six buttons, screen, buzzer and vibration motor.

    Buttons are ``up``, ``down``, ``left``, ``right``, ``a``, ``b`` - poll
    with ``is_pressed`` or attach ``when_pressed`` / ``when_released``
    handlers, which fire from hardware interrupts.
    """

    def __init__(self, brightness=20, gamma=1.0):
        self.up = Button(UP_PIN, "up")
        self.down = Button(DOWN_PIN, "down")
        self.left = Button(LEFT_PIN, "left")
        self.right = Button(RIGHT_PIN, "right")
        self.a = Button(A_PIN, "a")
        self.b = Button(B_PIN, "b")
        self.buttons = {b.name: b for b in
                        (self.up, self.down, self.left, self.right, self.a, self.b)}
        self.screen = Screen(brightness=brightness, gamma=gamma)
        self.buzzer = Buzzer()
        self.vibration = Vibration()

        # Original Kitronik API
        self.Up, self.Down, self.Left, self.Right = self.up, self.down, self.left, self.right
        self.A, self.B = self.a, self.b
        self.Screen, self.Buzzer, self.Vibrate = self.screen, self.buzzer, self.vibration

    def menu(self, items, **kwargs):
        """Show a Menu of ``items`` and return the selected index (or None)."""
        return Menu(self, items, **kwargs).run()

    def wait_for_press(self):
        """Block until any button is pressed, then return its name."""
        while True:
            for name, button in self.buttons.items():
                if button.is_pressed:
                    return name
            sleep_ms(10)


# Original Kitronik API
KitronikZIP96 = ZIP96
