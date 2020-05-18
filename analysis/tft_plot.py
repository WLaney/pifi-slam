#!/usr/bin/env python3
# Luke Puchalla, lcp77
# William Laney, wel53
import sys, pygame
import RPi.GPIO as GPIO
import time
import os

class TFTplotting:
    def __init__(self, file_paths):

        # GPIO set up
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)


        # set up exit interup

        # set up piTFT stuff
        os.putenv('SDL_VIDEODRIVER', 'fbcon')
        os.putenv('SDL_FBDEV', '/dev/fb1')
        os.putenv('SDL_MOUSEDRV', 'TSLIB')
        os.putenv('SDL_MOUSEDEV', '/dev/input/touchscreen')

        # set up pygames, must set up TFT first
        pygame.init()

        # set up interupts for buttons that change displaued plot
        # move 1 plot foward
        GPIO.add_event_detect(23, GPIO.FALLING,
                callback=lambda pin: self.change_plot(1), bouncetime=200)
        # move 1 plot backwards
        GPIO.add_event_detect(22, GPIO.FALLING,
                callback=lambda pin: self.change_plot(-1), bouncetime=200)
        GPIO.add_event_detect(27, GPIO.FALLING, callback=self.leave)
        # hide the mouse
        pygame.mouse.set_visible(False)

        self.size = width, height = 320, 240
        self.black = 0, 0, 0
        self.white = 255, 255, 255

        self.screen = pygame.display.set_mode(self.size)

        # load in the plots, hopefully we dont have enough to fill our memory
        self.loaded_plots = [0] * len(file_paths)
        self.loaded_plots_rects = [0] * len(file_paths)
        for i, plot_path in enumerate(file_paths):
            self.loaded_plots[i] = pygame.image.load(plot_path)
            self.loaded_plots_rects[i] = self.loaded_plots[i].get_rect(center = (160, 120))

        self.max_plot_ind = len(file_paths) - 1
        self.current_plot_ind = 0

        self.plot = self.loaded_plots[self.current_plot_ind]
        self.plotrec = self.loaded_plots_rects[self.current_plot_ind]

    # call back for exit pin
    def leave(self, pin):
        pygame.quit()
        GPIO.cleanup()
        quit()

    def show_plots(self):
        start = time.time()
        run_time = 0
        self.screen.blit(self.plot, self.plotrec)
        pygame.display.flip()
        while run_time < 90:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

            # get the current plot and rec
            self.plot = self.loaded_plots[self.current_plot_ind]
            self.plotrec = self.loaded_plots_rects[self.current_plot_ind]
            # update the screen
            self.screen.fill(self.white)            
            self.screen.blit(self.plot, self.plotrec)
            pygame.display.flip()
            run_time = time.time() - start

    def change_plot(self, amount):
        new_plot_ind = self.current_plot_ind + amount
        # loop if we're out of bounds
        if new_plot_ind < 0:
            self.current_plot_ind = self.max_plot_ind
        elif new_plot_ind > self.max_plot_ind:
            self.current_plot_ind = 0
        else:
            self.current_plot_ind = new_plot_ind




