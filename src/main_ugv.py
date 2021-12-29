#!/usr/bin/env python
# encoding:utf-8
import os
import rospy
from gettext import find
from os.path import split
from karmasim_ros_wrapper.msg import CarControls
from karmasim_dev_pkg.msg import SampleMessage
import copy
import select
import sys
import time
import tty
import math

import rospy
from geometry_msgs.msg import Pose
from karmasim_ros_wrapper.msg import CarControls
from karmasim_ros_wrapper.msg import UxvStates, PointsOfInterest
from karmasim_ros_wrapper.srv import VehicleStart, VehicleStop, VehicleUnload, VehicleLoad

from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String,UInt8,Float64
MAX_THROTTLE = 1.0
MAX_STEERING = 3

THROTTLE_STEP_SIZE = 0.5
STEERING_STEP_SIZE = 0.1
hizlanma_katsayisi = 0.2

uxv_pose_ = Pose()
uxv_radar_front = Range()
uxv_radar_rear = Range()
uxv_imu = Imu()
import copy

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------

__all__     = ['Munkres', 'make_cost_matrix', 'DISALLOWED']

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

# Info about the module
__version__   = "1.0.12"
__author__    = "Brian Clapper, bmc@clapper.org"
__url__       = "http://software.clapper.org/munkres/"
__copyright__ = "(c) 2008-2017 Brian M. Clapper"
__license__   = "Apache Software License"

# Constants
class DISALLOWED_OBJ(object):
    pass
DISALLOWED = DISALLOWED_OBJ()
DISALLOWED_PRINTVAL = "D"

# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class UnsolvableMatrix(Exception):
    """
    Exception raised for unsolvable matrices
    """
    pass

# ---------------------------------------------------------------------------
# Classes
# ---------------------------------------------------------------------------

class Munkres:
    """
    Calculate the Munkres solution to the classical assignment problem.
    See the module documentation for usage.
    """

    def __init__(self):
        """Create a new instance"""
        self.C = None
        self.row_covered = []
        self.col_covered = []
        self.n = 0
        self.Z0_r = 0
        self.Z0_c = 0
        self.marked = None
        self.path = None

    def make_cost_matrix(profit_matrix, inversion_function):
        """
        **DEPRECATED**

        Please use the module function ``make_cost_matrix()``.
        """

    make_cost_matrix = staticmethod(make_cost_matrix)

    def pad_matrix(self, matrix, pad_value=0):
        """
        Pad a possibly non-square matrix to make it square.

        :Parameters:
            matrix : list of lists
                matrix to pad

            pad_value : int
                value to use to pad the matrix

        :rtype: list of lists
        :return: a new, possibly padded, matrix
        """
        max_columns = 0
        total_rows = len(matrix)

        for row in matrix:
            max_columns = max(max_columns, len(row))

        total_rows = max(max_columns, total_rows)

        new_matrix = []
        for row in matrix:
            row_len = len(row)
            new_row = row[:]
            if total_rows > row_len:
                # Row too short. Pad it.
                new_row += [pad_value] * (total_rows - row_len)
            new_matrix += [new_row]

        while len(new_matrix) < total_rows:
            new_matrix += [[pad_value] * total_rows]

        return new_matrix

    def compute(self, cost_matrix):
        """
        Compute the indexes for the lowest-cost pairings between rows and
        columns in the database. Returns a list of (row, column) tuples
        that can be used to traverse the matrix.

        :Parameters:
            cost_matrix : list of lists
                The cost matrix. If this cost matrix is not square, it
                will be padded with zeros, via a call to ``pad_matrix()``.
                (This method does *not* modify the caller's matrix. It
                operates on a copy of the matrix.)

                **WARNING**: This code handles square and rectangular
                matrices. It does *not* handle irregular matrices.

        :rtype: list
        :return: A list of ``(row, column)`` tuples that describe the lowest
                 cost path through the matrix

        """
        self.C = self.pad_matrix(cost_matrix)
        self.n = len(self.C)
        self.original_length = len(cost_matrix)
        self.original_width = len(cost_matrix[0])
        self.row_covered = [False for i in range(self.n)]
        self.col_covered = [False for i in range(self.n)]
        self.Z0_r = 0
        self.Z0_c = 0
        self.path = self.__make_matrix(self.n * 2, 0)
        self.marked = self.__make_matrix(self.n, 0)

        done = False
        step = 1

        steps = { 1 : self.__step1,
                  2 : self.__step2,
                  3 : self.__step3,
                  4 : self.__step4,
                  5 : self.__step5,
                  6 : self.__step6 }

        while not done:
            try:
                func = steps[step]
                step = func()
            except KeyError:
                done = True

        # Look for the starred columns
        results = []
        for i in range(self.original_length):
            for j in range(self.original_width):
                if self.marked[i][j] == 1:
                    results += [(i, j)]

        return results

    def __copy_matrix(self, matrix):
        """Return an exact copy of the supplied matrix"""
        return copy.deepcopy(matrix)

    def __make_matrix(self, n, val):
        """Create an *n*x*n* matrix, populating it with the specific value."""
        matrix = []
        for i in range(n):
            matrix += [[val for j in range(n)]]
        return matrix

    def __step1(self):
        """
        For each row of the matrix, find the smallest element and
        subtract it from every element in its row. Go to Step 2.
        """
        C = self.C
        n = self.n
        for i in range(n):
            vals = [x for x in self.C[i] if x is not DISALLOWED]
            if len(vals) == 0:
                # All values in this row are DISALLOWED. This matrix is
                # unsolvable.
                raise UnsolvableMatrix(
                    "Row {0} is entirely DISALLOWED.".format(i)
                )
            minval = min(vals)
            # Find the minimum value for this row and subtract that minimum
            # from every element in the row.
            for j in range(n):
                if self.C[i][j] is not DISALLOWED:
                    self.C[i][j] -= minval
        return 2

    def __step2(self):
        """
        Find a zero (Z) in the resulting matrix. If there is no starred
        zero in its row or column, star Z. Repeat for each element in the
        matrix. Go to Step 3.
        """
        n = self.n
        for i in range(n):
            for j in range(n):
                if (self.C[i][j] == 0) and \
                        (not self.col_covered[j]) and \
                        (not self.row_covered[i]):
                    self.marked[i][j] = 1
                    self.col_covered[j] = True
                    self.row_covered[i] = True
                    break

        self.__clear_covers()
        return 3

    def __step3(self):
        """
        Cover each column containing a starred zero. If K columns are
        covered, the starred zeros describe a complete set of unique
        assignments. In this case, Go to DONE, otherwise, Go to Step 4.
        """
        n = self.n
        count = 0
        for i in range(n):
            for j in range(n):
                if self.marked[i][j] == 1 and not self.col_covered[j]:
                    self.col_covered[j] = True
                    count += 1

        if count >= n:
            step = 7 # done
        else:
            step = 4

        return step

    def __step4(self):
        """
        Find a noncovered zero and prime it. If there is no starred zero
        in the row containing this primed zero, Go to Step 5. Otherwise,
        cover this row and uncover the column containing the starred
        zero. Continue in this manner until there are no uncovered zeros
        left. Save the smallest uncovered value and Go to Step 6.
        """
        step = 0
        done = False
        row = 0
        col = 0
        star_col = -1
        while not done:
            (row, col) = self.__find_a_zero(row, col)
            if row < 0:
                done = True
                step = 6
            else:
                self.marked[row][col] = 2
                star_col = self.__find_star_in_row(row)
                if star_col >= 0:
                    col = star_col
                    self.row_covered[row] = True
                    self.col_covered[col] = False
                else:
                    done = True
                    self.Z0_r = row
                    self.Z0_c = col
                    step = 5

        return step

    def __step5(self):
        """
        Construct a series of alternating primed and starred zeros as
        follows. Let Z0 represent the uncovered primed zero found in Step 4.
        Let Z1 denote the starred zero in the column of Z0 (if any).
        Let Z2 denote the primed zero in the row of Z1 (there will always
        be one). Continue until the series terminates at a primed zero
        that has no starred zero in its column. Unstar each starred zero
        of the series, star each primed zero of the series, erase all
        primes and uncover every line in the matrix. Return to Step 3
        """
        count = 0
        path = self.path
        path[count][0] = self.Z0_r
        path[count][1] = self.Z0_c
        done = False
        while not done:
            row = self.__find_star_in_col(path[count][1])
            if row >= 0:
                count += 1
                path[count][0] = row
                path[count][1] = path[count-1][1]
            else:
                done = True

            if not done:
                col = self.__find_prime_in_row(path[count][0])
                count += 1
                path[count][0] = path[count-1][0]
                path[count][1] = col

        self.__convert_path(path, count)
        self.__clear_covers()
        self.__erase_primes()
        return 3

    def __step6(self):
        """
        Add the value found in Step 4 to every element of each covered
        row, and subtract it from every element of each uncovered column.
        Return to Step 4 without altering any stars, primes, or covered
        lines.
        """
        minval = self.__find_smallest()
        events = 0 # track actual changes to matrix
        for i in range(self.n):
            for j in range(self.n):
                if self.C[i][j] is DISALLOWED:
                    continue
                if self.row_covered[i]:
                    self.C[i][j] += minval
                    events += 1
                if not self.col_covered[j]:
                    self.C[i][j] -= minval
                    events += 1
                if self.row_covered[i] and not self.col_covered[j]:
                    events -= 2 # change reversed, no real difference
        if (events == 0):
            raise UnsolvableMatrix("Matrix cannot be solved!")
        return 4

    def __find_smallest(self):
        """Find the smallest uncovered value in the matrix."""
        minval = sys.maxsize
        for i in range(self.n):
            for j in range(self.n):
                if (not self.row_covered[i]) and (not self.col_covered[j]):
                    if self.C[i][j] is not DISALLOWED and minval > self.C[i][j]:
                        minval = self.C[i][j]
        return minval


    def __find_a_zero(self, i0=0, j0=0):
        """Find the first uncovered element with value 0"""
        row = -1
        col = -1
        i = i0
        n = self.n
        done = False

        while not done:
            j = j0
            while True:
                if (self.C[i][j] == 0) and \
                        (not self.row_covered[i]) and \
                        (not self.col_covered[j]):
                    row = i
                    col = j
                    done = True
                j = (j + 1) % n
                if j == j0:
                    break
            i = (i + 1) % n
            if i == i0:
                done = True

        return (row, col)

    def __find_star_in_row(self, row):
        """
        Find the first starred element in the specified row. Returns
        the column index, or -1 if no starred element was found.
        """
        col = -1
        for j in range(self.n):
            if self.marked[row][j] == 1:
                col = j
                break

        return col

    def __find_star_in_col(self, col):
        """
        Find the first starred element in the specified row. Returns
        the row index, or -1 if no starred element was found.
        """
        row = -1
        for i in range(self.n):
            if self.marked[i][col] == 1:
                row = i
                break

        return row

    def __find_prime_in_row(self, row):
        """
        Find the first prime element in the specified row. Returns
        the column index, or -1 if no starred element was found.
        """
        col = -1
        for j in range(self.n):
            if self.marked[row][j] == 2:
                col = j
                break

        return col

    def __convert_path(self, path, count):
        for i in range(count+1):
            if self.marked[path[i][0]][path[i][1]] == 1:
                self.marked[path[i][0]][path[i][1]] = 0
            else:
                self.marked[path[i][0]][path[i][1]] = 1

    def __clear_covers(self):
        """Clear all covered matrix cells"""
        for i in range(self.n):
            self.row_covered[i] = False
            self.col_covered[i] = False

    def __erase_primes(self):
        """Erase all prime markings"""
        for i in range(self.n):
            for j in range(self.n):
                if self.marked[i][j] == 2:
                    self.marked[i][j] = 0

# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------

def make_cost_matrix(profit_matrix, inversion_function=None):
    """
    Create a cost matrix from a profit matrix by calling
    'inversion_function' to invert each value. The inversion
    function must take one numeric argument (of any type) and return
    another numeric argument which is presumed to be the cost inverse
    of the original profit. In case the inversion function is not provided,
    calculate it as max(matrix) - matrix.

    This is a static method. Call it like this:

    .. python:

        cost_matrix = Munkres.make_cost_matrix(matrix, inversion_func)

    For example:

    .. python:

        cost_matrix = Munkres.make_cost_matrix(matrix, lambda x : sys.maxsize - x)

    :Parameters:
        profit_matrix : list of lists
            The matrix to convert from a profit to a cost matrix

        inversion_function : function
            The function to use to invert each entry in the profit matrix.
            In case it is not provided, calculate it as max(matrix) - matrix.

    :rtype: list of lists
    :return: The converted matrix
    """
    if not inversion_function:
        maximum = max(max(row) for row in profit_matrix)
        inversion_function = lambda x: maximum - x

    cost_matrix = []
    for row in profit_matrix:
        cost_matrix.append([inversion_function(value) for value in row])
    return cost_matrix

def print_matrix(matrix, msg=None):
    """
    Convenience function: Displays the contents of a matrix of integers.

    :Parameters:
        matrix : list of lists
            Matrix to print

        msg : str
            Optional message to print before displaying the matrix
    """

    if msg is not None:
        print(msg)

    # Calculate the appropriate format width.
    width = 0
    for row in matrix:
        for val in row:
            if val is DISALLOWED:
                val = DISALLOWED_PRINTVAL
            width = max(width, len(str(val)))

    # Make the format string
    format = ('%%%d' % width)

    # Print the matrix
    for row in matrix:
        sep = '['
        for val in row:
            if val is DISALLOWED:
                formatted = ((format + 's') % DISALLOWED_PRINTVAL)
            else: formatted = ((format + 'd') % val)
            sys.stdout.write(sep + formatted)
            sep = ', '
        sys.stdout.write(']\n')

kavsaklar = []
alfabe = ['0123456789.-wp']
alfabe1 = ['wp0123456789']
komsu = []
kavsakanaliz = []


def harfsor(harf):
    harfvarmi = 0
    for s in range(len(alfabe[0])):
        if harf == alfabe[0][s]:
            harfvarmi = 1
            break
        else:
            harfvarmi = 0
    return harfvarmi


def degistir(degisecek,alphabet):

    sayi = ''
    if degisecek != '' or degisecek != ' ':
        for i in range(len(degisecek)):
            for s in range(len(alphabet[0])):
                if degisecek[i] == alphabet[0][s]:
                    sayi += degisecek[i]
    return sayi


def readFile(filename):
    global komsu
    valu = []

    filehandle = open(filename)
    a = filehandle.readlines()
    filehandle.close()
    for i in range(len(a)):
        if a[i] == "NOKTALAR:\n":

            while i != len(a):
                i = i + 1
                if i == len(a):
                    break
                else:
                    yeni = a[i].split(" ", 4)
                    for k in range(len(yeni)):
                        yeni[k] = degistir(yeni[k], alfabe)
                        if yeni[k] != '':
                            valu.append(yeni[k])
                        if len(valu) == 4:
                            kavsaklar.append(valu)
                            valu = []
    for i in range(len(a)):
        if a[i] == "KOMŞULUKLAR:\n":
            while i != len(a):
                i = i + 1
                if i == len(a):
                    break
                elif a[i] == "NOKTALAR:\n":
                    break
                else:
                    yeni = a[i].split("-", 2)
                    for k in range(len(yeni)):
                        yeni[k] = degistir(yeni[k], alfabe1)
                        komsu.append(yeni[k])
                        if len(komsu) == 2:
                            kavsakanaliz.append(komsu)
                            komsu = []
    for i in kavsaklar:
        i[3]=float(i[3])
        i[1]=float(i[1])
        i[2]=float(i[2])
        a=i[0]
        i.pop(0)
        i.append(a)



kavsaklar = [[85, -142.6, -0.5, "wp1"],
             [85, -92.6, -0.5, "wp2"],
             [85, 77.2, -0.5, "wp3"],
             [29.9, 77.2, -0.5, "wp4"],
             [-75, 77.2, -0.5, "wp5"],
             [-129.9, 77.2, -0.5, "wp6"],
             [-129.9, -27.8, -0.5, "wp7"],
             [-129.9, -142.8, -0.5, "wp8"],
             [30.1, -142.8, -0.5, "wp9"],
             [30.1, -92.6, -0.5, "wp10"],
             [30.1, -27.7, -0.5, "wp11"],
             [30.1, 27.5, -0.5, "wp12"],
             [-25, 27.5, -0.5, "wp13"],
             [-25, -27.6, -0.5, "wp14"],
             [-75, -27.6, -0.5, "wp15"],
             [0, -27.7, -0.5, "wp16"],
             [6.4, -12.2, -0.5, "wp17"],
             [0, -12.2, -0.5, "wp18"],
             [-6.4, -12.2, -0.5, "wp19"]]
kavsakanaliz = [["wp1", "wp2"],
                ["wp1", "wp9"],
                ["wp2", "wp3"],
                ["wp2", "wp10"],
                ["wp3", "wp4"],
                ["wp4", "wp5"],
                ["wp4", "wp12"],
                ["wp5", "wp6"],
                ["wp5", "wp15"],
                ["wp6", "wp7"],
                ["wp7", "wp8"],
                ["wp7", "wp15"],
                ["wp8", "wp9"],
                ["wp9", "wp10"],
                ["wp10", "wp11"],
                ["wp11", "wp12"],
                ["wp11", "wp16"],
                ["wp12", "wp13"],
                ["wp13", "wp14"],
                ["wp14", "wp15"],
                ["wp14", "wp16"],
                ["wp16", "wp17"],
                ["wp16", "wp18"],
                ["wp16", "wp19"], ]
class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxint
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])
class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost=0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous
def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return
import heapq
def dijkstra(aGraph, start, target):

    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(), v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        # for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                # print ('updated : current = %s next = %s new_dist = %s' \
                #      %(current.get_id(), next.get_id(), next.get_distance()))
        #            else:
        # print ('not updated : current = %s next = %s new_dist = %s' \
        #      %(current.get_id(), next.get_id(), next.get_distance()))

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(), v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

def readFile(filename):
    filehandle = open(filename)
    kavsak_isimleri=filehandle.read()
    filehandle.close()

def kavsakuzak():
    # bagli kavsaklar arasi mesafe
    global kavsakanaliz
    global mesafekavsak
    for a in range(len(kavsakanaliz)):
        degisken1 = kavsakanaliz[a][0]
        degisken2 = kavsakanaliz[a][1]
        for b in range(len(kavsaklar)):
            if kavsaklar[b][3] == degisken1:
                degisken1_knm_x = kavsaklar[b][0]
                degisken1_knm_y = kavsaklar[b][1]
            if kavsaklar[b][3] == degisken2:
                degisken2_knm_x = kavsaklar[b][0]
                degisken2_knm_y = kavsaklar[b][1]
        mesafe_kavsak = abs(degisken1_knm_x - degisken2_knm_x) + abs(degisken1_knm_y - degisken2_knm_y)
        kavsakanaliz[a] = [kavsakanaliz[a][0], kavsakanaliz[a][1], str(mesafe_kavsak)]

def uxv_states_callback(data):
    global tehlike_list
    global ugv_list
    global hover
    global uxv_pose_
    global ika_konumlar
    global yakit_kapasite
    global kargo_agirligi
    global kargo_kapasite
    global kalan_yakit
    global kargo
    global yuk_liste
    ugv_list = []
    ika_konumlar = []
    kalan_yakit = []
    yakit_kapasite = []
    kargo_kapasite = []
    kargo_agirligi = []
    tehlike_list = []
    hover=0
    for uxv in data.uxvs:
        if uxv.name == uxv_name:
            uxv_pose_ = uxv.pose
    for uxv in data.uxvs:
        if uxv.name != uxv_name :
            mesafe = abs(uxv.pose.position.x - uxv_pose_.position.x) + abs(
                uxv.pose.position.y - uxv_pose_.position.y) + abs(uxv.pose.position.z - uxv_pose_.position.z)
            if mesafe < 10 and uxv.state == 2:
                tehlike_list.append(uxv.name)
    for eleman in tehlike_list:
        if int(eleman[-1]) < int(uxv_name[-1]):
            hover = 1
    yuk_liste = []
    for uxv in data.uxvs:
        if uxv.name[0:3] == 'ugv' and uxv.is_unavailable==0 :

            ugv_list.append(uxv.name)
            kargo = uxv.cargo
            kalan_yakit.append(uxv.fuel_current)
            yakit_kapasite.append(uxv.FUEL_CAPACITY)
            kargo_kapasite.append(uxv.CARGO_CAPACITY)
            kargo_agirligi.append(uxv.cargo_weight)
            ika_konumlar.append([uxv.pose.position.x, uxv.pose.position.y, uxv.pose.position.z])

def radar_front_callback(data):
    global uxv_radar
    uxv_radar_front = data.range

def radar_front_callback(data):
    global uxv_radar_front
    uxv_radar_front = data.range

def radar_rear_callback(data):
    global uxv_radar_rear
    uxv_radar_rear = data.range

def imu_callback(data):
    global uxv_imu
    global bas_acisi
    global eksen

    uxv_imu = data.orientation.w

    x = uxv_pose_.orientation.x
    y = uxv_pose_.orientation.y
    z = uxv_pose_.orientation.z
    ww = uxv_pose_.orientation.w

    t0 = +2.0 * (ww * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (ww * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (ww * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    bas_acisi = yaw_z
    global eksen_acisi
    if bas_acisi < -1.35 and bas_acisi > -1.75:
        eksen_acisi = -1.57
    elif bas_acisi < -2.95 or bas_acisi>3.05:
        eksen_acisi = 3.14
    elif bas_acisi > 1.40 and bas_acisi < 1.75:
        eksen_acisi = 1.57
    elif bas_acisi > -0.45 and bas_acisi < 0.25:
        eksen_acisi = 0

    if eksen_acisi==1.57 or eksen_acisi== -1.57:
        eksen="y"
    if eksen_acisi==0 or eksen_acisi== 3.14:
        eksen="x"

def ileri(hiz):
    global target_throttle
    global status
    if uxv_radar_front>5*hiz+5 and hover==0:
        car_cmd.brake = 0
        target_throttle = hiz
        if target_throttle > 0:
            car_cmd.manual = False
            if car_cmd.manual_gear != 1:
                car_cmd.manual_gear = 1
        else:
            target_throttle = 0
            car_cmd.brake = 1
            status = status + 1
    else:
        fren()

def geri(hiz):
    global target_throttle
    global status
    if uxv_radar_rear>5*hiz+8 and hover==0:
        car_cmd.brake = 0
        target_throttle = -hiz
        if target_throttle < 0:
            car_cmd.manual = True
            car_cmd.manual_gear = -1
        else:
            target_throttle = 0
            car_cmd.brake = 1
        status = status + 1
    else:
        fren()
def saga():
    global target_steering
    global status
    target_steering = MAX_STEERING
    status = status + 1

def sola():
    global target_steering
    global status
    target_steering = -MAX_STEERING
    status = status + 1

def topla():
    global target_steering
    global status
    target_steering = 0
    status = status + 1

def fren():
    global target_throttle
    global target_steering
    global status
    target_throttle = 0
    target_steering = 0
    car_cmd.brake = 1
    status = status + 1

def pose(uav_pose):
    return uav_pose.position
def vels(target_throttle, target_steering, target_break, target_handbreak):
    return "currently:\t throttle %s\t steering %s\t break %s\t handbreak %s " % (
        target_throttle, target_steering, target_break, target_handbreak)
def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input
def checkLimitThrottle(vel):
    vel = constrain(vel, -MAX_THROTTLE, MAX_THROTTLE)
    return vel
def checkLimitSteering(vel):
    vel = constrain(vel, -MAX_STEERING, MAX_STEERING)
    return vel
yer_knm = []
kavsaklar_arasi = []

def list_states_callback(data):
    global hasta_listesi
    global patient_list
    global merkezler
    global hastaneler
    global gereksinim
    global istasyonlar
    global patient_konumlar
    global patient_uzunluk
    global hastane_kota
    patient_list = []
    hastaneler = []
    merkezler = []
    hasta_listesi = []
    gereksinim = []
    istasyonlar = []
    hastane_kota=[]

    patient_konumlar = []
    for PointOfInterest in data.pois:
        patient_list.append(PointOfInterest.name)
        patient_konumlar.append([PointOfInterest.pose.position.x, PointOfInterest.pose.position.y,
                                 PointOfInterest.pose.position.z])
        if PointOfInterest.name[0:3] == "pat" and PointOfInterest.requirement == 3:
            hasta_listesi.append(PointOfInterest.name)
            gereksinim.append(PointOfInterest.requirement)
        if PointOfInterest.name[0:3] == 'hos':
            hastaneler.append(PointOfInterest.name)
            hastane_kota.append(PointOfInterest.current_quotas)
        if PointOfInterest.name[0:3] == 'cha':
            istasyonlar.append(PointOfInterest.name)
    patient_uzunluk = len(patient_list)


def kavsaklararasi():
    global kavsaklar_arasi

    for y in range(len(patient_konumlar)):
        for x in range(len(kavsaklar)):
            if (kavsaklar[x][0] - 1.5) < patient_konumlar[y][0] < (kavsaklar[x][0] + 1.5):
                kavsaklar_arasi.append([kavsaklar[x][3], patient_list[y], (
                        abs(kavsaklar[x][0] - patient_konumlar[y][0]) + abs(kavsaklar[x][1] - patient_konumlar[y][1]))])
            if (kavsaklar[x][1] - 1.5) < patient_konumlar[y][1] < (kavsaklar[x][1] + 1.5):
                kavsaklar_arasi.append([kavsaklar[x][3], patient_list[y], (
                        abs(kavsaklar[x][0] - patient_konumlar[y][0]) + abs(kavsaklar[x][1] - patient_konumlar[y][1]))])

def sagadon():
    if eksen_acisi==3.14:
        don_deger=1.57
    elif eksen_acisi==-1.57:
        don_deger=0
    elif eksen_acisi==0:
        don_deger=1.57
    elif eksen_acisi==1.57:
        don_deger=3.14
    a=1
    while a>0.015:

        saga()
        a=abs(abs(bas_acisi)-don_deger)
        a=a**3
        if a>0.99:
            a=0.98
        ileri(a/2.8)
        car_cmd.throttle = target_throttle
        car_cmd.steering = target_steering
        pub.publish(car_cmd)
    fren()

def soladon():
    if eksen_acisi==-1.57:
        don_deger=3.14
    elif eksen_acisi==3.14:
        don_deger=1.57
    elif eksen_acisi==1.57:
        don_deger=0
    elif eksen_acisi==0:
        don_deger=1.57
    a=1
    while a>0.015:
        sola()
        a=abs(abs(bas_acisi)-don_deger)
        a=a**3
        if a>0.99:
            a=0.98
        ileri(a/2.8)
        car_cmd.throttle = target_throttle
        car_cmd.steering = target_steering
        pub.publish(car_cmd)
    fren()

def yuk_bosalt(hastane_adi):
    cargo_type = 5
    target_poi = hastane_adi
    amount_number = 1
    rospy.wait_for_service('/karmasim_node/vehicle_unload')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_unload', VehicleUnload)
    res = req(uxv_name, target_poi, cargo_type, amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))

def yuk_yukle(hasta_adi):
    cargo_type = 5
    source_poi = hasta_adi
    amount_number = 1
    rospy.wait_for_service('/karmasim_node/vehicle_load')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_load', VehicleLoad)
    res = req(uxv_name, source_poi ,cargo_type , amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))

def atama(hstnlr,srlm,hstn_kt,hst_lsts,ugv_lst):
    global ika_rota_konum
    gorev_alinmadi=1
    ika_rota_konum=[]
    dis_matris=[]
    for i in hst_lsts:
        ic_matris=[]
        indis=-1
        for j in ugv_lst:
            indis+=1
            ic_matris.append(ika_yollar1[srlm.index(i)][srlm.index(ugv_konumlar[indis])])
        dis_matris.append(ic_matris)
    if len(dis_matris)>0:
        m = Munkres()
        atama_liste=m.compute(dis_matris)
        atamalar=[]
        for atama in range(len(atama_liste)):
            atamalar.append([hst_lsts[atama],ugv_lst[atama_liste[atama][1]]])
        ika_hedef=[]
        for kontrol in atamalar:
            if kontrol[1]==uxv_name:
                ika_hedef.append(kontrol[0])
                gorev_alinmadi=0

        if gorev_alinmadi!=1:
            mesafe=9999999
            for hastane in hstnlr:
                if mesafe>ika_yollar1[srlm.index(hastane)][srlm.index(ika_hedef[0])] and hstn_kt[hstnlr.index(hastane)]>0:
                    mesafe=ika_yollar1[srlm.index(hastane)][srlm.index(ika_hedef[0])]
                    belirlenmis_hastane=hastane
            ika_hedef.append(belirlenmis_hastane)

            ika_hedef.insert(0,ugv_konumlar[ugv_lst.index(uxv_name)])
            ika_rota=[]
            for i in range(len(ika_hedef)):
                if i >0:
                    ika_rota=ika_rota+(ara_yollar[((srlm.index(ika_hedef[i-1]))*len(srlm))+srlm.index(ika_hedef[i])])
                    dongu=-1
            for i in ika_rota:
                dongu+=1
                if dongu>0:
                    if i==ika_rota[dongu-1]:
                        ika_rota.pop(dongu-1)
            kavsak_indis=[]
            for kavsak_isim in kavsaklar:
                kavsak_indis.append(kavsak_isim[3])

            for konum in ika_rota:
                if konum[0:2]=="wp":
                    ika_rota_konum.append([kavsaklar[kavsak_indis.index(konum)][0],kavsaklar[kavsak_indis.index(konum)][1],0])
                elif konum[0:2]=="pa":
                    ika_rota_konum.append([patient_konumlar[patient_list.index(konum)][0],patient_konumlar[patient_list.index(konum)][1],1,konum])
                else:
                    ika_rota_konum.append([patient_konumlar[patient_list.index(konum)][0],patient_konumlar[patient_list.index(konum)][1],2,konum])

def ugv_konum(ika_knmlr,patient_knmlr,patient_lst):
    global ugv_konumlar
    noktalar=[]
    nokta_isimleri=[]
    ugv_konumlar=[]
    j=-1
    for kavsak in kavsaklar:
        noktalar.append([kavsak[0],kavsak[1],kavsak[2]])
        nokta_isimleri.append(kavsak[3])
    for konumlar in patient_knmlr:
        j+=1
        noktalar.append(konumlar)
        nokta_isimleri.append(patient_lst[j])

    for ugv in ika_knmlr:
        indis=-1
        toplam_mesafe=100000
        for yer_knm in noktalar:
            indis+=1
            if toplam_mesafe>abs(yer_knm[0]-ugv[0])+abs(yer_knm[1]-ugv[1])+abs(yer_knm[2]-ugv[2]):
                toplam_mesafe=abs(yer_knm[0]-ugv[0])+abs(yer_knm[1]-ugv[1])+abs(yer_knm[2]-ugv[2])
                ugv_nokta=nokta_isimleri[indis]
        ugv_konumlar.append(ugv_nokta)

def ileriye():
    dongu=1
    while dongu!=2:
        x_sapma=abs(uxv_pose_.position.x-ika_rota_konum[0][0])
        y_sapma=abs(uxv_pose_.position.y-ika_rota_konum[0][1])
        sapma=x_sapma+y_sapma
        hizlanma = abs(1/(-math.log(sapma / (86.3 - (-151.6)), 2)))


        car_cmd.throttle = target_throttle
        pub.publish(car_cmd)
        if (eksen_acisi==-1.57 and uxv_pose_.position.y < ika_rota_konum[0][1]) or \
                (eksen_acisi==1.57 and uxv_pose_.position.y > ika_rota_konum[0][1])  or \
                (eksen_acisi==3.14 and uxv_pose_.position.x < ika_rota_konum[0][0]) or \
                (eksen_acisi==0 and uxv_pose_.position.x > ika_rota_konum[0][0]) :
            fren()
            dongu = 2
            pub.publish(car_cmd)
        else:
            ileri(hizlanma)
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            pub.publish(car_cmd)
def geriye():
    dongu=1
    while dongu!=2:
        x_sapma=abs(uxv_pose_.position.x-ika_rota_konum[0][0])
        y_sapma=abs(uxv_pose_.position.y-ika_rota_konum[0][1])
        sapma=x_sapma+y_sapma
        hizlanma = abs(1/(-math.log(sapma / (86.3 - (-151.6)), 2)))
        car_cmd.throttle = target_throttle
        pub.publish(car_cmd)
        if (eksen_acisi==-1.57 and uxv_pose_.position.y > ika_rota_konum[0][1]) or (eksen_acisi==1.57 and uxv_pose_.position.y < ika_rota_konum[0][1])  or (eksen_acisi==3.14 and uxv_pose_.position.x > ika_rota_konum[0][0]) or (eksen_acisi==0 and uxv_pose_.position.x < ika_rota_konum[0][0]) :
            fren()
            dongu = 2
            pub.publish(car_cmd)
        else:
            geri(hizlanma)
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            pub.publish(car_cmd)



if __name__ == "__main__":
    rospy.init_node('contester_ugv_node')

    #Relative path example
    script_dir = os.path.dirname(__file__)
    rel_path = '../res/ydc_formal.txt'
    abs_file_path = os.path.join(script_dir, rel_path)
    readFile(abs_file_path)

    pub = rospy.Publisher('ugv_cmd', CarControls, queue_size=10)
    sample_msg_pub = rospy.Publisher('/karmasim_node/sample_message', SampleMessage, queue_size=10)
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]

    status = 0
    target_throttle = 0.0
    target_steering = 0.0

    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True

    uxv_name = rospy.get_param('~uxv_name', 'ugv_1')
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]
    rospy.Subscriber('/karmasim_node/uxv_states', UxvStates, uxv_states_callback)
    rospy.Subscriber('/karmasim_node/points_of_interest', PointsOfInterest, list_states_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_front_center', Range, radar_front_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_rear_center', Range, radar_rear_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/imu/Imu', Imu, imu_callback)


try:
    rospy.wait_for_service('/karmasim_node/vehicle_start')
    request = rospy.ServiceProxy('/karmasim_node/vehicle_start', VehicleStart)
    result = request(uxv_name)

    while 1:
        time.sleep(0.5)
        ugv_konum(ika_konumlar,patient_konumlar,patient_list)
        kavsakuzak()
        kavsaklararasi()
        g = Graph()
        vertex = []
        kavsaks = []
        for i in patient_list:
            vertex.append(i)
        for i in kavsaklar:
            vertex.append(i)
        for i in range(len(vertex)):
            g.add_vertex(vertex[i][3])
        for i in kavsaklar_arasi:
            kavsaks.append(i)
        for i in kavsakanaliz:
            kavsaks.append(i)
        for i in range(len(kavsaks)):
            g.add_edge(kavsaks[i][0], kavsaks[i][1], float(kavsaks[i][2]))
        for v in g:
            for w in v.get_connections():
                vid = v.get_id()
                wid = w.get_id()
        ika_yollar = []
        ika_yollar1 = []
        siralama = []
        ara_yollar=[]
        for k in g.vert_dict:
            ika_yollar = []
            for j in g.vert_dict:
                gt = copy.deepcopy(g)
                if (k[0:3]=="pat" or k[0:3]=="hos" or k[0:2]=="wp") and (j[0:3]=="pat" or j[0:3]=="hos" or j[0:2]=="wp"):
                    dijkstra(gt, gt.get_vertex(k), gt.get_vertex(j))
                    # print(g.get_vertices())
                    target = gt.get_vertex(j)
                    path = [target.get_id()]
                    shortest(target, path)
                    # print('The shortest path : %s' % (path[::-1]))
                    kl=(path[::-1])
                    ara_yollar.append(kl)
                    en_kisa_mesafe = 0
                    for i in range(len(path) - 1):
                        en_kisa_mesafe += (gt.get_vertex(path[i]).adjacent[gt.get_vertex(path[i + 1])])
                    ika_yollar.append(en_kisa_mesafe)
            if k[0:3]=="pat" or k[0:3]=="hos" or k[0:2]=="wp":
                siralama.append(k)
                ika_yollar1.append(ika_yollar)

        atama(hastaneler,siralama,hastane_kota,hasta_listesi,ugv_list)


        while len(ika_rota_konum)>0:
            x_sapma=abs(uxv_pose_.position.x-ika_rota_konum[0][0])
            y_sapma=abs(uxv_pose_.position.y-ika_rota_konum[0][1])
            if x_sapma+y_sapma>8:

                if (eksen=="y" and y_sapma>3) or (eksen=="x" and x_sapma>3) :
                    if (eksen_acisi==3.14 and ika_rota_konum[0][0]<uxv_pose_.position.x) or (eksen_acisi==0 and ika_rota_konum[0][0]>uxv_pose_.position.x) or (eksen_acisi==-1.57 and ika_rota_konum[0][1]<uxv_pose_.position.y) or (eksen_acisi==1.57 and ika_rota_konum[0][1]>uxv_pose_.position.y):

                        ileriye()
                    elif (eksen_acisi==3.14 and ika_rota_konum[0][0]>uxv_pose_.position.x) or (eksen_acisi==0 and ika_rota_konum[0][0]<uxv_pose_.position.x) or (eksen_acisi==-1.57 and ika_rota_konum[0][1]>uxv_pose_.position.y) or (eksen_acisi==1.57 and ika_rota_konum[0][1]<uxv_pose_.position.y):
                        geriye()
                elif (eksen=="x" and y_sapma>5) or (eksen=="y" and x_sapma>5) :
                    if (eksen_acisi==3.14 and ika_rota_konum[0][1]>uxv_pose_.position.y) or (eksen_acisi==0 and ika_rota_konum[0][1]<uxv_pose_.position.y) or (eksen_acisi==-1.57 and ika_rota_konum[0][0]<uxv_pose_.position.x) or (eksen_acisi==1.57 and ika_rota_konum[0][0]>uxv_pose_.position.x):
                        soladon()
                    elif (eksen_acisi==3.14 and ika_rota_konum[0][1]<uxv_pose_.position.y) or (eksen_acisi==0 and ika_rota_konum[0][1]>uxv_pose_.position.y) or (eksen_acisi==-1.57 and ika_rota_konum[0][0]>uxv_pose_.position.x) or (eksen_acisi==1.57 and ika_rota_konum[0][0]<uxv_pose_.position.x):
                        sagadon()
            else:
                if ika_rota_konum[0][2]==1:
                    yuk_yukle(ika_rota_konum[0][3])
                elif ika_rota_konum[0][2]==2:
                    yuk_bosalt(ika_rota_konum[0][3])
                ika_rota_konum.pop(0)
                time.sleep(0.5)

            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            pub.publish(car_cmd)
except:
    a=1
finally:
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    car_cmd.throttle = 0
    car_cmd.steering = 0
    car_cmd.brake = 0
    pub.publish(car_cmd)
    request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
    result = request(uxv_name)