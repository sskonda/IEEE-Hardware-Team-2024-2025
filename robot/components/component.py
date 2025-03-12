import pigpio
import sys

class Component():
    def __init__(self):
        self.initialized = False

    def init(self, pi: pigpio.pi) -> bool:
        """Initializes the component with the given pigpio instance
        """
        try:
            success = self._init(pi)
        except Exception as e:
            print(f"Failed to initialize component: {e}", file=sys.stderr)
            success = False
        self.initialized = success
        return success

    def _init(self, pi: pigpio.pi) -> bool:
        raise NotImplementedError()
    
    def update(self):
        pass

    def release(self):
        """Releases the hardward resources to drive the component
        """
        raise NotImplementedError()
    
    def __del__(self):
        self.release()