from numpy import pi
from Boid import Boid, NamedPipe
import p5


with NamedPipe('r') as pipe:

    def setup():
        p5.size(Boid.SIM_SIZE.x, Boid.SIM_SIZE.y)

    def draw():

        for _ in range(5):
            all_boids = pipe.recive()

        Boid.DrawAllBoids(p5, all_boids)

    p5.run(setup, draw)