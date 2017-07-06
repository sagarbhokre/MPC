import pyautogui, time

#print (pyautogui.locateOnScreen('previous.png'))
'''
try:
   while True:
       # TODO: Get and print the mouse coordinates.
       x, y = pyautogui.position()
       positionStr = 'X: ' + str(x).rjust(4) + ' Y: ' + str(y).rjust(4)
       print(positionStr)
except KeyboardInterrupt:
    print('\nDone.')
    exit(0)
'''

def restart_game():
    x, y = pyautogui.position()
    pyautogui.click(1320,360);
    pyautogui.typewrite(['esc']);
    time.sleep(2);
    pyautogui.click(1320,360);
    time.sleep(2);
    pyautogui.click(1530,440);
    time.sleep(1);
    pyautogui.click(x,y);

restart_game()
