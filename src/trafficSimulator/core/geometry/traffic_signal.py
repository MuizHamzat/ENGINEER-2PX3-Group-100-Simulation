class TrafficSignal:
    def __init__(self, segments, config={}):
        # Initialize segments
        self.segments = segments
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)
        # Calculate properties
        self.init_properties()

    def set_default_config(self):
        self.cycle = [(False, True,15), (False,False,16.5), (True, False,31.5), (False, False, 33)]
        # Cycle times were multiplied by a factor of 1/4: 60, 66, 126, 132 -> 15, 16.5, 31.5, 33
        self.slow_distance = 40
        self.slow_factor = 10
        self.stop_distance = 15

        self.current_cycle_index = 0
        self.last_t = 0
    

    def init_properties(self):
        for i in range(len(self.segments)):
            for segment in self.segments[i]:
                segment.set_traffic_signal(self, i)

    @property
    def current_cycle(self):
        return self.cycle[self.current_cycle_index]

    def update(self, sim):
      
        
        k = int(sim.t) % self.cycle[-1][2]
        for i in range(len(self.cycle)):
            if k < self.cycle[i][2]:
                self.current_cycle_index=i
                break


        