class Component():
    def init(self, pi):
        """Acquires the hardware resources to drive the component
        """
        raise NotImplementedError()
    
    def update(self):
        pass

    def release(self):
        """Releases the hardward resources to drive the component
        """
        raise NotImplementedError()
    
    def __del__(self):
        self.release()