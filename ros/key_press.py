#!/usr/bin/env python
import curses

MENU = 'Use the arrow keys to drive: Up/Down = Throttle, Left/Right = Steering\n'

def main(stdscr):
    THROTTLE = 0.0
    STEERING = 0.0

    # do not wait for input when calling getch
    #stdscr.nodelay(1)
    stdscr.addstr(MENU)
    while True:
        # get keyboard input, returns -1 if none available
        c = stdscr.getch()
        if c != -1:
            if c == 259: # Up
                THROTTLE = min(THROTTLE + 10.0, 100.0)
            elif c == 258: # Down
                THROTTLE = max(THROTTLE - 10.0, -100.0)
            elif c == 260: # Left
                STEERING = max(STEERING - 10.0, -100.0)
            elif c == 261: # Right
                STEERING = min(STEERING + 10.0, 100.0)
            # print numeric value
            stdscr.move(0, 0)
            stdscr.addstr(MENU)
            stdscr.addstr('THROTTLE: {}    STEERING: {}\n'.format(THROTTLE, STEERING))
            stdscr.refresh()
            # return curser to start position
            stdscr.move(0, 0)

if __name__ == '__main__':
    curses.wrapper(main)
