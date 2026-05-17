import sys
import tty
import termios
import threading


################### DOESNT WORK ###################

class KeyboardController:
    def __init__(self):
        self.current_key = None
        self.running = True
        
        # Start the listener in a background thread
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

    def _getch(self):
        """Reads a single character from the terminal instantly."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # Set terminal to raw mode (no echo, no waiting for Enter)
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            # Always restore the terminal settings, even if it crashes
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def _listen(self):
        """Background loop waiting for keystrokes."""
        print("WASD to move, Q to stop/decelerate, Ctrl+C to quit.")
        while self.running:
            key = self._getch().lower() # Convert to lowercase for easy checking
            
            if key == '\x03': # Ctrl+C was pressed
                self.running = False
                print("\nExiting...")
                return
                
            self.current_key = key

    def get_last_key(self):
        """Returns the last pressed key and clears it."""
        key = self.current_key
        self.current_key = None # Clear it so we don't read it twice
        return key

    def stop(self):
        self.running = False