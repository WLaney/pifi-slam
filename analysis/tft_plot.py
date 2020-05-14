#!/usr/bin/env python3
# Luke Puchalla, lcp77
# William Laney, wel53
import sys, pygame
import RPi.GPIO as GPIO
import time
import os

# GPIO set up
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# call back for exit pin
def leave(pin):
    GPIO.cleanup()
    quit()
# set up exit interup
GPIO.add_event_detect(27, GPIO.FALLING, callback=leave)

# set up piTFT stuff
#os.putenv('SDL_VIDEODRIVER', 'fbcon')
#os.putenv('SDL_FBDEV', '/dev/fb1')
#os.putenv('SDL_MOUSEDRV', 'TSLIB')
#os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

class TFTplotting:
    def __init__(self, file_path):
        # set up pygames, must set up TFT first
        pygame.init()

        # hide the mouse
        pygame.mouse.set_visible(False)

        self.size = width, height = 320, 240
        self.black = 0, 0, 0

        self.screen = pygame.display.set_mode(self.size)

        self.plot = pygame.image.load(file_path)
        self.plotrec = self.plot.get_rect()

    def show_plots(self):
        start = time.time()
        run_time = 0
        self.screen.blit(self.plot, self.plotrec)
        pygame.display.flip()
        while run_time < 60:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

            #self.screen.fill(self.black)
            #screen.blit(self.plot, self.plotrec)
            #pygame.display.flip()
            run_time = time.time() - start
