"""
Demo for the zip96 library on the Kitronik ZIP96 Retro Gamer.

Copy zip96.py, picozero.py and this file to the Pico, then run it.
Up/Down move through the menu, A selects, B backs out of a demo.
"""

from time import sleep_ms

from zip96 import ZIP96, BLACK, RED, GREEN, CYAN, PURPLE, WHITE

gamer = ZIP96(brightness=15)
screen = gamer.screen


def text_demo():
    screen.scroll_text("HELLO FROM ZIP96!", CYAN, stop=gamer.b.pressed)


def rainbow_demo():
    """Diagonal colour wave until B is pressed."""

    def wheel(pos):
        pos %= 255
        if pos < 85:
            return (255 - pos * 3, pos * 3, 0)
        if pos < 170:
            pos -= 85
            return (0, 255 - pos * 3, pos * 3)
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)

    shift = 0
    while not gamer.b.is_pressed:
        for y in range(screen.height):
            for x in range(screen.width):
                screen.set_pixel(x, y, wheel((x + y) * 12 + shift))
        screen.show()
        shift += 6
        sleep_ms(40)


def sprite_demo():
    """A beating heart sprite, drawn with blit(). B exits."""
    heart = [".r.r.",
             "rrrrr",
             "rrrrr",
             ".rrr.",
             "..r.."]
    big = True
    while not gamer.b.is_pressed:
        screen.clear()
        if big:
            screen.blit(heart, 3, 1, {"r": RED})
        else:
            screen.blit(heart[1:4], 4, 2, {"r": PURPLE})
        screen.show()
        gamer.vibration.buzz(0.05)
        big = not big
        sleep_ms(400)


def tune_demo():
    screen.centre_text(">>>", 1, GREEN)
    screen.show()
    gamer.buzzer.play([("c4", 0.2), ("e4", 0.2), ("g4", 0.2), ("c5", 0.4),
                       ("g4", 0.2), ("c5", 0.6)])


# Event-driven input: handlers fire from hardware interrupts, no polling loop.
gamer.a.when_pressed = lambda: gamer.vibration.buzz(0.05)

DEMOS = [("TEXT", text_demo),
         ("RAINBOW", rainbow_demo),
         ("HEART", sprite_demo),
         ("TUNE", tune_demo)]

screen.scroll_text("ZIP96", WHITE, step_ms=60)

while True:
    choice = gamer.menu([name for name, _ in DEMOS])
    if choice is None:
        screen.centre_text("BYE", 1, RED)
        screen.show()
        sleep_ms(800)
        screen.clear(show=True)
        break
    DEMOS[choice][1]()
    screen.clear(show=True)
