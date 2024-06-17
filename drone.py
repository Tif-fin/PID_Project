import math


class Drone:
    def __init__(self,altitude=0, velocity=0, weight=1.0) -> None:
        self.altitude = altitude
        self.velocity = velocity 
        self.weight = weight 
        self.__gravity = 9.8
    
    def update(self,thrust,dt):
        gravity = self.__gravity * self.weight
        acceleration = (thrust-gravity)/self.weight
        self.velocity += acceleration*dt 
        self.altitude += self.velocity*dt


