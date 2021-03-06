#!/usr/bin/python
##
## Menu system
##
## Menu is list of menu items with following format :
## [reference to parent entry identifier,
##  menu entry display title,
##  positive integer identifier (free choice) if entry goes to submenu if selected OR
##  string with action to be returned when entry is selected]
##
## Below MyMenu and MyMenu2 examples will result in same MENU !
##
'''Test Code'''

'''
MyMenu = [
    [0, "Menu Top", 1],
    [1, "Pompe", 2],
    [2, "Auto", "exec pompe auto"],
    [2, "ON", "exec pompe on"],
    [2, "OFF", "exec pompe OFF"],
    [1, "Lampe", 3],
    [3, "ON", "exec Lampe ON"],
    [3, "OFF", "exec Lampe OFF"],
    [1, "Config", 4],
    [4, "IP", 5],
    [5, "SET", "exec IP Set"],
    [4, "WIFI", 6],
    [6, "Check", "exec wifi check"],
    [6, "Show", "exec wifi show"]
]
'''
MyMenu = [
    # Top Header --> Level 0
    [0, "AutoSmoker", 1], # Top Level Menu Title

    # Top Level Menu --> Level 1
    [1, "Display Current Temps", 2], # Top level Menu, Choice 1.2
    [1, "Smoker Temp Set", 3], # Top level Menu, Choice 1.3
    [1, "Meat Alarm Temp", 4], # Top level Menu, Choice 1.4
    [1, "Config", 5], # Top level menu, Choice 1.5


    # Sub-menu --> From Choice 2
    [2, "Auto 2", "exec pompe auto 2"], #Sub-menu Choice 2.1
    [2, "ON 2", "exec pompe on 2"], #Sub-menu, Choice 2.2
    [2, "OFF 2", "exec pompe OFF 2"], #Sub-menu, Choice 2.3

    # Sub-menu --> From Choice 3
    [3, "ON 2", "exec Lampe ON 2"], #Sub-menu, Choice 3.1
    [3, "OFF 2", "exec Lampe OFF 2"], #Sub-menu, Choice 3.2

    # Sub-menu 1.3
    [4, "IP 2", 5], #Sub-menu 1.3, Choice 1
    [4, "WIFI 2", 6], #Sub-menu 1.3, Choice 2

    # Sub-men 2
    [5, "SET 2", "exec IP Set 2"],

    [6, "Check 2", "exec wifi check 2"],
    [6, "Show 2", "exec wifi show 2"]
]


def getch():
    """
    Reads one character from keyboard without need for pressing RETURN.
    Only works on POSIX/LINUX systems
    """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class menu:
    """
    Menu System typically for 2 line LCD and 2 button navigation.
    structure is list of menu items with following format :
    [reference to parent entry identifier,
    menu entry display title,
    positive integer identifier (free choice) if entry goes to submenu if selected OR
    string with action to be returned when entry is selected]
    """

    def __init__(self, structure, ExitLabel="EXIT"):

        # Initialize internal variables
        self._menu = []
        i = 0
        r = {0: 0}

        # Copy menu adding indexes
        for item in structure:
            if isinstance(item[2], int): r[item[2]] = i
            self._menu.append([i, r[item[0]], item[1], item[2]])
            i += 1
        self.key = 1
        self.niveau = 1

        # Build internal dictionnary representing menu structure
        self.struct = {}
        for item in self._menu:
            if item[0] != 0:
                if item[1] not in self.struct: self.struct[item[1]] = []
                self.struct[item[1]].append(item[0])

        # Add EXIT entry for each level
        for key in self.struct:
            index = len(self._menu)
            self._menu.append([index, key, ExitLabel, -1])
            self.struct[key].append(index)

    def display(self, pos=None):
        """
        Displays the current menu entry as :
        1st line : parent entry
        2nd line : current selected item
        """
        if pos is None: pos = self.key
        self.key = pos
        parent = self._menu[pos][1]
        print str(self.niveau) + ":" + self._menu[parent][2]
        print self._menu[pos][2]

    def action(self, pos=None):
        """
        Returns action associated with current selected menu item
        None if selected item refers to submenu
        """
        if pos is None: pos = self.key
        self.key = pos
        if isinstance(self._menu[pos][3], str): return self._menu[pos][3]
        return None

    def next(self, pos=None):
        """
        Advances selection to next menu item in same level.
        Rotates to first item when at end of menu level.
        """
        if pos is None: pos = self.key
        self.key = pos
        level = self.struct[self._menu[pos][1]]
        i = level.index(pos)
        i = i + 1
        if i >= len(level): i = 0
        self.key = level[i]

    def select(self, pos=None):
        """
        Advances to sub menu or parent menu then returns False, OR
        Returns True if selected menu has to be actioned.
        Returns -1 if selected item is EXIT from first level, i.e. quit the menu
        """
        if pos is None: pos = self.key
        self.key = pos
        if isinstance(self._menu[pos][3], str): return True
        if self._menu[pos][3] > 0:
            self.key = self.struct[self._menu[pos][0]][0]
            self.niveau += 1
            return False
        if self._menu[pos][3] == -1:
            self.key = self._menu[pos][1]
            self.niveau -= 1
            if self.key == 0: return -1
            return False

    def up(self, pos=None):
        """
        Goes up one level and returns True
        Returns False if already on top/first level
        Can typically be used after actionning a selection to return to previous menu item
        """
        if pos is None: pos = self.key
        self.key = pos
        if self.niveau == 1: return False
        self.key = self._menu[pos][1]
        self.niveau -= 1
        return True


if __name__ == "__main__":

    import os

    m = menu(MyMenu)
    action = ""

    while True:
        os.system("clear")
        m.display()
        print action
        c = getch()
        if c == "x": break
        if c == "n": m.next()  # Simulate NEXT button
        if c == "s":  # Simulate SELECT button
            s = m.select()
            if s:
                if s == -1: break
                action = m.action()
                m.up()

