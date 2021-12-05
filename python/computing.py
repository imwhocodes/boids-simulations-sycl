from Boid import Boid, NamedPipe
import time


print('Init!')

all_boids = Boid.GenerateRandomBoids(1500)

# with NamedPipe('w') as pipe:

print('Starting!')

while True:
    start_time = time.time()
    all_boids = Boid.ComputeUpdate(all_boids)
    # pipe.send(all_boids)
    delta = time.time() - start_time

    print(delta, '\t\t', 1 / delta)



    

