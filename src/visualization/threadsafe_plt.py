import threading

import matplotlib

matplotlib.use('TkAgg')
from matplotlib import pyplot as plt


class ThreadsafePlotManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._initialized = False
        
    def create_figure(self, *args, **kwargs):
        with self._lock:
            if not self._initialized:
                # Enable interactive mode
                plt.ion()
                self._initialized = True
            
            return plt.figure(*args, **kwargs)
            
    def close_all(self):
        with self._lock:
            plt.ioff()  # Disable interactive mode
            plt.close('all')

    def initialize_display(self):
        """Initialize matplotlib display in main thread"""
        with self._lock:
            if not self._initialized:
                plt.ion()
                self._initialized = True
                plt.show()
                plt.pause(0.1)  # Give window time to initialize

    def process_events(self):
        """Process matplotlib events in main thread"""
        with self._lock:
            plt.pause(0.01)

    def finalize_display(self):
        """Clean up display when simulation is complete"""
        with self._lock:
            plt.ioff()
            plt.show(block=True)  # Block until window is closed

plot_manager = ThreadsafePlotManager() 