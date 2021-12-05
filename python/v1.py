
import itertools
import p5

from Boid import Boid


all_boids = None



def setup():
    global all_boids
    p5.size(Boid.SIM_SIZE.x, Boid.SIM_SIZE.y) #instead of create_canvas
    all_boids = Boid.GenerateRandomBoids(1000)

def draw():
    global all_boids

    all_boids = Boid.ComputeUpdate(all_boids)

    Boid.DrawAllBoids(all_boids)



p5.run()