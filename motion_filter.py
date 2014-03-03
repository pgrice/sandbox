#!/usr/bin/env python

import pygame
import random
import time


N_Particles = 1000
X_MAX = 640
Y_MAX = 480
X_NOISE = 10
Y_NOISE = 10

def predict(x,y,dx,dy):
    pass

def update(x,y,dx,dy):
    pass

def get_random_point(xmax=X_MAX, ymax=Y_MAX):
    return (random.randint(0, xmax-1), random.randint(0, ymax-1))

def update_particles(plist, dx, dy):
    for part in plist:
        part[0] += dx
        part[1] += dy
    for part in plist:
        if part
    return plist


def main():
    screen = pygame.display.set_mode((X_MAX, Y_MAX));
    pygame.mouse.set_visible(False)

    particles = []
    for i in range(N_Particles):
        particles.append(get_random_point())

    mousex, mousey = (X_MAX/2, Y_MAX/2)
    lasttime = time.time()
    period = 0.01
    running = True
    while running:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEMOTION:
            mousex, mousey = event.pos
            dx, dy = event.rel

       particles = update_particles(particles, dx, dy)

        x = mousex
        y = mousey
        x += random.randint(-X_NOISE, X_NOISE)
        if x <0:
            x = 0
        elif x > X_MAX:
            x = X_MAX
        y += random.randint(-Y_NOISE, Y_NOISE)
        if y < 0:
            y = 0
        elif y > Y_MAX:
            y = Y_MAX

        screen.fill((0,0,0))
        pygame.draw.circle(screen, (255,255,255), (x,y), 3)
        pygame.display.flip()
        now = time.time()
        tdiff = now - lasttime
        slptm = period - tdiff
        if slptm > 0.:
            time.sleep(slptm)
        lasttime = now




if __name__=='__main__':
    main()
