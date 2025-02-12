import argparse
import logging
import os
import sys
import threading
from pathlib import Path
import signal

import matplotlib
matplotlib.use('TkAgg')  # Set here to ensure thread safety

import matplotlib.pyplot as plt

# Add parent directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drone_sim.core.engine import SimulationEngine
from drone_sim.systems.visualization.matplotlib import MatplotlibVisualizer

import logging
logger = logging.getLogger(__name__)

def setup_logging(debug_mode=False):
    """Centralized logging configuration for all modules"""
    log_level = logging.DEBUG if debug_mode else logging.INFO
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    
    logging.basicConfig(
        level=log_level,
        format=log_format,
        datefmt='%H:%M:%S',
        handlers=[
            logging.StreamHandler(sys.stdout),
        ]
    )

    # Override specific loggers
    noisy_loggers = {
        'matplotlib': logging.WARNING,
        'PIL.PngImagePlugin': logging.WARNING,
        'urllib3': logging.WARNING
    }
    
    for logger_name, level in noisy_loggers.items():
        logging.getLogger(logger_name).setLevel(level)

    if debug_mode:
        # Enable debug for project modules
        debug_loggers = [
            'core',
            'systems',
            'models',
            'utils'
        ]
        for name in debug_loggers:
            logging.getLogger(name).setLevel(logging.DEBUG)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/simulation_config.json')
    parser.add_argument('--fast', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--avoid-collisions', default='basic')
    args = parser.parse_args()
    
    # Setup logging before anything else
    setup_logging(args.debug)
    logger.info("Starting simulation with debug=%s", args.debug)
    
    # Load and validate config
    config_path = args.config
    logger.info("Loading configuration from: %s", config_path)
    if not Path(config_path).exists():
        logger.error("Config file not found: %s", config_path)
        sys.exit(1)
    
    # Create engine in main thread
    try:
        engine = SimulationEngine.from_config(config_path)
        logger.info("Engine initialized successfully")
    except Exception as e:
        logger.error("Failed to initialize engine: %s", e, exc_info=True)
        sys.exit(1)
    
    # Add event bus debugging
    if args.debug:
        def event_logger(event_name, data):
            logger.debug("EVENT: %s (Data keys: %s)", event_name, data.keys())
        engine.event_bus.subscribe_all(event_logger)
    
    def handle_interrupt(sig, frame):
        logging.info("\nReceived shutdown signal...")
        if engine:
            engine.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, handle_interrupt)
    
    # Simulation runs in background thread
    sim_thread = threading.Thread(target=engine.run, daemon=True)
    sim_thread.start()
    
    # Visualization runs in main thread
    if engine.systems['visualization']:
        engine.systems['visualization'].start_gui_loop()
    
    # Keep window open after simulation completes
    if not args.fast and engine.systems.get('visualization'):
        plt.show(block=True)

    # Ensure engine and visualizer share the SAME event bus
    logger.info("Engine event bus ID: %s", id(engine.event_bus))
    logger.info("Visualizer event bus ID: %s", 
              id(engine.systems['visualization'].event_bus))

    # Add thread verification
    logger.info("Main thread: %s", threading.current_thread().name)
    logger.info("Visualizer thread: %s", 
              engine.systems['visualization'].thread.name)

if __name__ == "__main__":
    main()