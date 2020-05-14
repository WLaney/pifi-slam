#!/usr/bin/env python3
# Luke Puchalla, lcp77
# William Laney, wel53
import sys, pygame
import RPi.GPIO as GPIO
import time
import os

# GPIO set up
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# call back for exit pin
def leave(pin):
    quit()
# set up exit interup
GPIO.add_event_detect(17, GPIO.FALLING, callback=leave)

# set up piTFT stuff
os.putenv('SDL_VIDEODRIVER', 'fbcon')
os.putenv('SDL_FBDEV', '/dev/fb1')
os.putenv('SDL_MOUSEDRV', 'TSLIB')
os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

# set up pygames, must set up TFT first
pygame.init()

# hide the mouse
pygame.mouse.set_visible(False)

size = width, height = 320, 240
black = 0, 0, 0

screen = pygame.display.set_mode(size)

plot = pygame.image.load("testing/test.png")
plotrec = plot.get_rect()
start = time.time()
run_time = 0
while run_time < 60:
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    screen.fill(black)
    screen.blit(plot, plotrec)
    pygame.display.flip()
    run_time = time.time() - start
