import random
import math
from numbers import Number
import os
import pickle
import struct
import collections

class NamedPipe():

    PIPE_FILENAME = '/tmp/boid_pipe'

    FMT = struct.Struct("<I")

    def __init__(self, mode):
        self.mode = mode

    def open(self):
        try:
            os.mkfifo(self.PIPE_FILENAME)
        except FileExistsError:
            print(self.PIPE_FILENAME, '\texisted!')
            pass

        if self.mode == 'w':
            print('Waiting for a READER')
            self.pipe = os.open(self.PIPE_FILENAME, os.O_WRONLY)

        elif self.mode == 'r':
            print('Waiting for a WRITER')
            self.pipe = os.open(self.PIPE_FILENAME, os.O_RDONLY)
            
        else:
            raise ValueError()

        os.set_blocking(self.pipe, True)

        print(self.PIPE_FILENAME, '\topened with mode:\t', self.mode)

    def close(self):
        print('CLOSING FILE!!!')
        os.close(self.pipe)


    def __enter__(self):
        self.open()
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def send(self, data):
        # assert(self.mode == 'w')

        pickled = pickle.dumps(data)

        size = len(pickled)

        header = self.FMT.pack(size)

        # print('\n{} -> {}\n'.format(header, size))

        os.write(self.pipe, header)
        os.write(self.pipe, pickled)

        
    def recive(self):
        # assert(self.mode == 'r')

        header = os.read(self.pipe, self.FMT.size)

        size = self.FMT.unpack(header)[0]

        # print('\n{} -> {}\n'.format(header, size))

        pickled = bytearray()
        still_size = size

        while still_size > 0:

            readed = os.read(self.pipe, still_size)

            pickled += readed

            still_size -= len(readed)

        return pickle.loads(pickled)


def constrain(in_v, min_v, max_v):
    return max(min(in_v, max_v), min_v)

class Vector2D(object):

    __slots__ = ('x', 'y')

    def __init__(self, x, y):
        """ Create a vector, example: v = Vector2D(1,2) """
        self.x = x
        self.y = y
            
    def norm(self):
        """ Returns the norm (length, magnitude) of the vector """
        return math.sqrt(self.x**2 + self.y**2)

    def distance(self, other):
        return (self - other).norm()


    def argument(self):
        """ Returns the argument of the vector, the angle clockwise from +y."""
        arg_in_rad = math.acos(self.__class__(0, 1) * self / self.norm())
        arg_in_deg = math.degrees(arg_in_rad)
        if self.values[0] < 0:
            return 360 - arg_in_deg
        else:
            return arg_in_deg

    def normalize(self):
        """ Returns a normalized unit vector """
        norm = self.norm()
        return self.__class__(self.x / norm, self.y / norm)

    def rotate(self, *args):
        """ Rotate this vector. If passed a number, assumes this is a
            2D vector and rotates by the passed value in degrees.  Otherwise,
            assumes the passed value is a list acting as a matrix which rotates the vector.
        """
        if len(args) == 1 and isinstance(args[0], Number):
            # So, if rotate is passed an int or a float...
            if len(self) != 2:
                raise ValueError("Rotation axis not defined for greater than 2D vector")
            return self._rotate2D(*args)
        elif len(args) == 1:
            matrix = args[0]
            if not all(len(row) == len(matrix) for row in matrix) or not len(matrix) == len(self):
                raise ValueError("Rotation matrix must be square and same dimensions as vector")
            return self.matrix_mult(matrix)

    def _rotate2D(self, theta):
        """ Rotate this vector by theta in degrees.

            Returns a new vector.
        """
        theta = math.radians(theta)
        # Just applying the 2D rotation matrix
        dc, ds = math.cos(theta), math.sin(theta)
        x, y = self.values
        x, y = dc*x - ds*y, ds*x + dc*y
        return self.__class__(x, y)

    def matrix_mult(self, matrix):
        """ Multiply this vector by a matrix.  Assuming matrix is a list of lists.

            Example:
            mat = [[1,2,3],[-1,0,1],[3,4,5]]
            Vector2D(1,2,3).matrix_mult(mat) ->  (14, 2, 26)

        """
        if not all(len(row) == len(self) for row in matrix):
            raise ValueError('Matrix must match vector dimensions')

        # Grab a row from the matrix, make it a Vector2D, take the dot product,
        # and store it as the first component
        product = tuple(self.__class__(*row) * self for row in matrix)

        return self.__class__(*product)

    def inner(self, other):
        """ Returns the dot product (inner product) of self and other vector
        """
        return sum(a * b for a, b in zip(self, other))

    def __mul__(self, other):
        """ Returns the dot product of self and other if multiplied
            by another Vector2D.  If multiplied by an int or float,
            multiplies each component by other.
        """
        if type(other) == type(self):
            return self.inner(other)
        elif isinstance(other, Number):
            return self.__class__(
                                    self.x * other,
                                    self.y * other
            )
        else:
            raise ValueError()

    def __rmul__(self, other):
        """ Called if 4*self for instance """
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, Number):
            return self.__class__(
                                    self.x / other,
                                    self.y / other
                                )
        elif isinstance(other, self.__class__):
            return self.__class__(
                        self.x / other.x,
                        self.y / other.y
                    )
        else:
            raise ValueError()
        
    def __floordiv__(self, other):
        if isinstance(other, Number):
            return self.__class__(
                                    self.x // other,
                                    self.y // other
                                )
        elif isinstance(other, self.__class__):
            return self.__class__(
                                    self.x // other.x,
                                    self.y // other.y
                    )
        else:
            raise ValueError()
        
    def __div__(self, other):
        return self.__truediv__(other)

    def __add__(self, other):
        """ Returns the vector addition of self and other """
        return self.__class__(
                                self.x + other.x,
                                self.y + other.y,
        )

    def __sub__(self, other):
        """ Returns the vector difference of self and other """
        return self.__class__(
                                self.x - other.x,
                                self.y - other.y,
        )

    def __iter__(self):
        return iter((self.x, self.y))

    def __len__(self):
        return 2

    @classmethod
    def Constrain(cls, val, min, max):
        return cls(
                    constrain(val.x, min.x, max.x),
                    constrain(val.y, min.y, max.y)
                )



class IDX():

    __slots__ = ('A', 'B')

    def __init__(self, A : int, B : int) -> None:
        self.A = A
        self.B = B

    def __hash__(self) -> int:
        return hash((self.A, self.B))

    def __eq__(self, other) -> bool:
        return self.A == other.A and self.B == other.B

    def __add__(self, other):
        return self.__class__(self.A + other.A, self.B + other.B)

class FastSearch2D():

    __slots__ = ('search_radius', 'map')
    
    def __init__(self, search_radius):
        self.search_radius = search_radius

        self.map = collections.defaultdict(list)

    def getSectorFromVect(self, vect):
        return self.getSectorFromCoord(vect.x, vect.y)

    def getSectorFromCoord(self, x, y):
        return IDX(
                    round(x / self.search_radius),
                    round(y / self.search_radius)
                )

    def addElementVect(self, e):
        self.map[self.getSectorFromVect(e.getPosition())].append(e)


    def searchInRadius(self, center) -> set:

        result = list()

        center_idx = self.getSectorFromVect(center)
        
        for x in range(-1, 2):
            for y in range(-1, 2):
                for e in self.map[ center_idx + IDX(x,y) ]:
                    if center.distance(e.getPosition()) < self.search_radius:
                        result.append(e)

        return result


class Actor(object):

    SIM_SIZE = Vector2D(1920, 1080)

    __slots__ = ('position')

    def __init__(self, pos):
        self.position = Vector2D.Constrain(pos, Vector2D(0,0), self.SIM_SIZE)

    def getPosition(self):
        return self.position


class Boid(Actor):

    BOID_SIZE = 5

    PERCEPTION = 150

    MAX_SPEED = 5

    __slots__ = ('velocity')

    def __init__(self, pos, vel):
        super().__init__(pos)
        self.velocity = vel
        # self.velocity.limit(None, self.MAX_SPEED)


    @classmethod
    def ApplyVelocity(cls, boid):
        return cls( boid.position + boid.velocity, boid.velocity)


    @classmethod
    def ApplyVelocityList(cls, boids):
        return [ cls.ApplyVelocity(b) for b in boids]

    # def show(self):
    #     self.P5.stroke(255)
    #     self.P5.circle(self.position.x, self.position.y, self.BOID_SIZE)

    @classmethod
    def GenerateRandomBoids(cls, n):
        return [ 
                    cls( 
                        Vector2D(
                                    random.random() * cls.SIM_SIZE.x,
                                    random.random() * cls.SIM_SIZE.y
                                ),
                        Vector2D(
                                    (random.random() * cls.MAX_SPEED) - (cls.MAX_SPEED / 2),
                                    (random.random() * cls.MAX_SPEED) - (cls.MAX_SPEED / 2)
                        )
                    )

                    for _ in range (n)
                ]
                                    
    @classmethod
    def DrawAllBoids(cls, p5, boids):
        p5.background(30, 30, 47)
        p5.stroke(255) 
        for b in boids:
            p5.circle(b.position.x, b.position.y, cls.BOID_SIZE)


    @classmethod
    def ComputeUpdate(cls, orig_boids):

        boids = cls.ApplyVelocityList(orig_boids)

        searcher = FastSearch2D(cls.PERCEPTION)

        for b in boids:
            searcher.addElementVect(b)


        # print(searcher.map)

        res = list()

        for b in boids:
            masked = searcher.searchInRadius(b.getPosition())

            if masked:

                cm = b.getOthersCenter(masked)

                al = b.getOthersAlignement(masked)

                res.append(cls(cm, al))

            else:
                res.append(b)


        return res
            
    
    def getMaskedByDistance(self, others):
        return [ o for o in others if ( self.position.distance( o.position ) < self.PERCEPTION ) ]


    def getOthersAlignement(self, others):

        new_speed = Vector2D(0, 0)

        for o in others:
            new_speed += o.velocity

        return new_speed / len(others)


    def getOthersCenter(self, others):

        center = Vector2D(0, 0)

        for o in others:
            center += o.position

        return center / len(others)