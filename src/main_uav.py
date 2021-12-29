#!/usr/bin/env python

import os
import time


from karmasim_ros_wrapper.msg import VelCmd
from karmasim_dev_pkg.msg import SampleMessage

import math
from karmasim_dev_pkg.msg import SampleMessage
from sensor_msgs.msg import Range, Imu
import select
import sys
import tty
import time

import rospy
from geometry_msgs.msg import Pose, Point
from karmasim_ros_wrapper.msg import VelCmd, UxvStates, Uxv, PointsOfInterest
from karmasim_ros_wrapper.srv import VehicleStart, VehicleStop, VehicleUnload, VehicleLoad

LINEAR_Z_MAX = 10
LINEAR_X_MAX = 10
LINEAR_Y_MAX = 10
ANGULAR_Z_MAX = 2

LINEAR_Z_STEP_SIZE = 1
LINEAR_X_STEP_SIZE = 1
LINEAR_Y_STEP_SIZE = 1
ANGULAR_Z_STEP_SIZE = 0.2

msg = """

"""

e = """
Communications Failed
"""
alinmis_hasta=[]
uxv_name = ""
uxv_pose_ = Pose()
poi_pose_ = Pose()
uxv_radar_front = Range()
uxv_radar_rear = Range()
uxv_radar_left = Range()
uxv_radar_right = Range()
uxv_radar_bottom = Range()
uxv_radar_top = Range()
kalan_yakit = Pose()
yakit_kapasite = Pose()
kargo_kapasite = Pose()
kargo_agirligi = Pose()
kargo = Pose()
hover = 0
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
    import math

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



def radar_front_callback(data):
    global uxv_radar_front
    uxv_radar_front = data.range


def radar_rear_callback(data):
    global uxv_radar_rear
    uxv_radar_rear = data.range


def radar_left_callback(data):
    global uxv_radar_left
    uxv_radar_left = data.range


def radar_right_callback(data):
    global uxv_radar_right
    uxv_radar_right = data.range


def radar_bottom_callback(data):
    global uxv_radar_bottom
    uxv_radar_bottom = data.range


def radar_top_callback(data):
    global uxv_radar_top
    uxv_radar_top = data.range



def vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed):
    return 'currently:\t linear_z %s\t linear_x %s\t linear_y %s\t angular_z %s ' % (
        target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed)


def pose(uav_pose):
    print('Position:')
    return uav_pose.position


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input


def checkLimitSpeed(vel, max):
    vel = constrain(vel, -max, max)
    return vel


def uxv_states_callback(data):
    global tehlike_list
    global uav_list
    global hover
    global uxv_pose_
    global iha_konumlar
    global yakit_kapasite
    global kargo_agirligi
    global kargo_kapasite
    global kalan_yakit
    global kargo
    global yuk_liste
    global anlik_x_hiz
    global anlik_y_hiz
    global anlik_z_hiz

    global yakin
    uav_list = []
    iha_konumlar = []
    kalan_yakit = []
    yakit_kapasite = []
    kargo_kapasite = []
    kargo_agirligi = []

    tehlike_list = []

    hover = 0
    for uxv in data.uxvs:
        if uxv.name == uxv_name:
            uxv_pose_ = uxv.pose
            anlik_x_hiz=abs(uxv.twist.linear.x)
            anlik_y_hiz=abs(uxv.twist.linear.y)
            anlik_z_hiz=abs(uxv.twist.linear.z)
    for uxv in data.uxvs:
        if uxv.name != uxv_name:
            mesafe = abs(uxv.pose.position.x - uxv_pose_.position.x) + abs(
                uxv.pose.position.y - uxv_pose_.position.y) + abs(uxv.pose.position.z - uxv_pose_.position.z)
            mesafe_xy =abs(uxv.pose.position.x - uxv_pose_.position.x) + abs(uxv.pose.position.y - uxv_pose_.position.y)

            if (mesafe < 15 and uxv.state == 2) or (mesafe_xy<6 and uxv.state == 2):
                tehlike_list.append(uxv.name)
    for eleman in tehlike_list:
        if int(eleman[-1]) > int(uxv_name[-1]):
            hover = 1
            yakin=1

    yuk_liste = []
    for uxv in data.uxvs:
        ilacsayisi=0
        erzaksayisi=0
        if uxv.name[0:3] == 'uav' and uxv.is_unavailable==0:
            uav_list.append(uxv.name)
            kargo = uxv.cargo
            ilacsayisi = kargo.count('\x01')
            erzaksayisi = kargo.count('\x02')
            yuk_liste.append([ilacsayisi, erzaksayisi])
            kalan_yakit.append(uxv.fuel_current)
            yakit_kapasite.append(uxv.FUEL_CAPACITY)
            kargo_kapasite.append(uxv.CARGO_CAPACITY)
            kargo_agirligi.append(uxv.cargo_weight)
            iha_konumlar.append([uxv.pose.position.x, uxv.pose.position.y, uxv.pose.position.z])




def imu_callback(data):
    global uxv_imu
    global bas_acisi
    global bas_acisi_sapma
    uxv_imu = uxv_pose_.orientation.w

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
    bas_acisi_sapma = 3.14 - abs(bas_acisi)

def atama_modul():
    global load_list
    global hedef_knm
    hedef_knm=[]
    if patient_kontrol==1:
        karsilastirma(patient_konumlar,uav_list,iha_konumlar)
        atanmis_uav=[]
        indis = -1
        hospitale_git = 0
        gidaya_git = 0
        en_kisa_gidis = 10000000000
        hasta_indis=-1
        matris=[]
        matris_sira=[]
        for patient in patient_list:
            indis += 1
            if hasta_listesi.count(patient)==1:
                hasta_indis += 1
                indis_uav = -1
                ic_matris=[]
                ic_matris_sira=[]
                for uav_lst in uav_list:
                    indis_uav += 1

                    en_kisa_gidis = 10000000000

                    if int(yuk_liste[indis_uav][int(gereksinim[hasta_indis])-1]) > 0:
                        for istasyon in istasyonlar:
                            t_mesafe = karsilastirilan[patient_uzunluk + indis_uav-1][indis] + karsilastirilan[indis][patient_list.index(istasyon)]
                            if en_kisa_gidis > t_mesafe:
                                en_kisa_gidis = t_mesafe
                                en_ykn_istasyon = istasyon
                        if kalan_yakit[indis_uav] - (t_mesafe / 4 * 200) > 0:

                            matris_deger = t_mesafe
                        else:
                            matris_deger = 99999999999

                    else:

                        if gereksinim[hasta_indis] == 1:

                            for hospit in hastaneler:
                                for istasyon in istasyonlar:
                                    t_mesafe = karsilastirilan[patient_uzunluk + indis_uav-1][patient_list.index(hospit)] + karsilastirilan[indis][patient_list.index(istasyon)] + karsilastirilan[patient_list.index(hospit)][indis]
                                    if en_kisa_gidis > t_mesafe:
                                        en_kisa_gidis = t_mesafe
                                        en_ykn_istasyon = istasyon
                                        en_ykn_hastane = hospit
                            if kalan_yakit[indis_uav] - (t_mesafe / 4 * 200) > 0:
                                matris_deger = t_mesafe
                            else:
                                matris_deger = 99999999999
                        else:
                            for merkez in merkezler:
                                for istasyon in istasyonlar:
                                    t_mesafe = karsilastirilan[patient_uzunluk + indis_uav-1][patient_list.index(merkez)] + karsilastirilan[indis][patient_list.index(istasyon)] + karsilastirilan[patient_list.index(merkez)][indis]
                                    if en_kisa_gidis > t_mesafe:
                                        en_kisa_gidis = t_mesafe
                                        en_ykn_istasyon = istasyon
                                        en_ykn_merkez = merkez
                            if kalan_yakit[indis_uav] - (t_mesafe / 4 * 200) > 0:
                                matris_deger = t_mesafe
                            else:
                                matris_deger = 99999999999

                    ic_matris.append(matris_deger)
                    ic_matris_sira.append(uav_lst)
                matris.append(ic_matris)
                matris_sira.append(patient)

        m = Munkres()
        atama_liste=m.compute(matris)

        atanmis_gorev=[]
        for kontrol in range(len(atama_liste)):
            atanmis_uav.append(atama_liste[kontrol][1])
            atanmis_gorev.append(atama_liste[kontrol][0])

        list_indis=-1
        iha_gorev=0
        for bumu in atanmis_uav:
            list_indis+=1

            if ic_matris_sira[bumu]==uxv_name:
                this_indis=list_indis
                iha_gorev=1

        load_list=[]
        if iha_gorev==1:
            atanan_hasta=matris_sira[atama_liste[this_indis][0]]

            ilgili_gereksinim=gereksinim[hasta_listesi.index(atanan_hasta)]
            if int(yuk_liste[uav_list.index(uxv_name)][int(ilgili_gereksinim)-1]) > 0:
                hedef_knm.append(patient_konumlar[patient_list.index(atanan_hasta)])
                load_list.append([2,atanan_hasta,ilgili_gereksinim])
            else:
                en_kisa_gidis=9999999
                if ilgili_gereksinim==1:
                    for hospit in hastaneler:
                        t_mesafe = karsilastirilan[patient_uzunluk + int(uav_list.index(uxv_name))-1][patient_list.index(hospit)] +  karsilastirilan[patient_list.index(hospit)][patient_list.index(atanan_hasta)]
                        if en_kisa_gidis > t_mesafe:
                            en_kisa_gidis = t_mesafe
                            en_ykn_hastane = hospit
                    hedef_knm.append(patient_konumlar[patient_list.index(en_ykn_hastane)])
                    load_list.append([1,en_ykn_hastane,1,2])
                    hedef_knm.append(patient_konumlar[patient_list.index(atanan_hasta)])
                    load_list.append([2,atanan_hasta,1])
                if ilgili_gereksinim==2:
                    for merkez in merkezler:
                        t_mesafe = karsilastirilan[patient_uzunluk + int(uav_list.index(uxv_name))-1][patient_list.index(merkez)] +  karsilastirilan[patient_list.index(merkez)][patient_list.index(atanan_hasta)]
                        if en_kisa_gidis > t_mesafe:
                            en_kisa_gidis = t_mesafe
                            en_ykn_merkez = hospit
                    hedef_knm.append(patient_konumlar[patient_list.index(en_ykn_merkez)])
                    load_list.append([1,en_ykn_merkez,2,2])
                    hedef_knm.append(patient_konumlar[patient_list.index(atanan_hasta)])
                    load_list.append([2,atanan_hasta,2])


def yuk_states_callback(data):
    global hasta_listesi
    global patient_list
    global merkezler
    global hastaneler
    global gereksinim
    global istasyonlar
    global patient_konumlar
    global patient_uzunluk
    global patient_kontrol
    patient_list = []
    hastaneler = []
    merkezler = []
    hasta_listesi = []
    gereksinim = []
    istasyonlar = []

    patient_konumlar = []
    patient_kontrol=0
    for PointOfInterest in data.pois:
        patient_list.append(PointOfInterest.name)
        patient_konumlar.append([PointOfInterest.pose.position.x, PointOfInterest.pose.position.y,
                                 PointOfInterest.pose.position.z])
        if PointOfInterest.name[0:3] == "pat" and PointOfInterest.requirement != 3:
            hasta_listesi.append(PointOfInterest.name)
            gereksinim.append(PointOfInterest.requirement)
        if PointOfInterest.name[0:3] == 'hos':
            hastaneler.append(PointOfInterest.name)
        if PointOfInterest.name[0:3] == 'hea':
            merkezler.append(PointOfInterest.name)
        if PointOfInterest.name[0:3] == 'cha':
            istasyonlar.append(PointOfInterest.name)
    patient_kontrol=1
    patient_uzunluk = len(patient_list)

def istasyona_mesafe():
    global eyi
    global hedef_knm
    global load_list
    en_yakin_istasyon = 100000
    for istasyonn in istasyonlar:
        sira = patient_list.index(istasyonn)
        ax = patient_konumlar[sira][0] - uxv_pose_.position.x
        ay = patient_konumlar[sira][1] - uxv_pose_.position.y
        az = patient_konumlar[sira][2] - uxv_pose_.position.z
        ax = abs(ax)
        ay = abs(ay)
        az = abs(az)
        toplam_mesafe = ((math.sqrt((ax ** 2) + (ay ** 2) + (az ** 2))) + 4 * (
                math.sqrt((ax ** 2) + (ay ** 2)) + (2 * az)) + (math.sqrt((ax ** 2) + (ay ** 2)) + (4 * az))) / 6
        if toplam_mesafe < en_yakin_istasyon:
            en_yakin_istasyon = toplam_mesafe
            eyi = istasyonn
    if kalan_yakit<6000:
        hedef_knm=[]
        load_list=[]
        hedef_knm.append(patient_konumlar[patient_list.index(eyi)])
        load_list.append([3,30])
def karsilastirma(list,uav_lst,iha_knm):
    global karsilastirilan
    karsilastirilan = []

    for ihalar in uav_lst:
        patient_list.append(ihalar)
    for iha_konum in iha_knm:
        list.append(iha_konum)
    dd=len(patient_list)
    for deger in range(dd):
        sozluk = []
        for deger1 in range(dd):
            ax = list[deger][0] - list[deger1][0]
            ay = list[deger][1] - list[deger1][1]
            az = list[deger][2] - list[deger1][2]
            ax = abs(ax)
            ay = abs(ay)
            az = abs(az)
            toplam_mesafe = ((math.sqrt((ax ** 2) + (ay ** 2) + (az ** 2))) + 4 * (
                    math.sqrt((ax ** 2) + (ay ** 2)) + (2 * az)) + (math.sqrt((ax ** 2) + (ay ** 2)) + (4 * az))) / 6
            sozluk.append(toplam_mesafe)
        karsilastirilan.append(sozluk)



def eksi_y_hareket():
    global target_lin_y_speed
    global target_lin_z_speed
    global yuksel
    if y_sapma > 20:
        if 38 < uxv_radar_right:
            target_lin_y_speed = -10
        elif 38 > uxv_radar_right > 20:
            yuksel = 1
            target_lin_y_speed = -10
        elif uxv_radar_right > 11:
            yuksel = 1
            target_lin_y_speed = -4
        else:
            yuksel = 1
            target_lin_y_speed = 0
    elif y_sapma > 1:
        target_lin_y_speed = -1.5
    else:
        target_lin_y_speed = 0

    if yuksel == 1:
        target_lin_z_speed = 10


def arti_y_hareket():
    global target_lin_y_speed
    global target_lin_z_speed
    global yuksel
    if y_sapma > 20:
        if 38 < uxv_radar_left:
            target_lin_y_speed = 10
        elif 38 > uxv_radar_left > 20:
            yuksel = 1
            target_lin_y_speed = 10
        elif uxv_radar_left > 11:
            yuksel = 1
            target_lin_y_speed = 4
        else:
            yuksel = 1
            target_lin_y_speed = 0
    elif y_sapma > 1:
        target_lin_y_speed = 1.5
    else:
        target_lin_y_speed = 0

    if yuksel == 1 and y_sapma > 10:
        target_lin_z_speed = 10


def arti_x_hareket():
    global target_lin_x_speed
    global target_lin_z_speed
    global yuksel
    if x_sapma > 20:
        if 38 < uxv_radar_rear:
            target_lin_x_speed = 10
        elif 38 > uxv_radar_rear > 20:
            yuksel = 1
            target_lin_x_speed = 10
        elif uxv_radar_rear > 11:
            yuksel = 1
            target_lin_x_speed = 4
        else:
            yuksel = 1
            target_lin_x_speed = 0
    elif x_sapma > 1:
        target_lin_x_speed = 1.5
    else:
        target_lin_x_speed = 0
    if yuksel == 1:
        target_lin_z_speed = 10


def eksi_x_hareket():
    global target_lin_x_speed
    global target_lin_z_speed
    global yuksel
    if x_sapma > 20:
        if 38 < uxv_radar_front:
            target_lin_x_speed = -10
        elif 38 > uxv_radar_front > 20:
            yuksel = 1
            target_lin_x_speed = -10
        elif uxv_radar_front > 11:
            yuksel = 1
            target_lin_x_speed = -4
        else:
            yuksel = 1
            target_lin_x_speed = 0
    elif x_sapma > 1:
        target_lin_x_speed = -1.5
    else:
        target_lin_x_speed = 0
    if yuksel == 1:
        target_lin_z_speed = 10

def load_fonksiyon(konum_adi,gereksinim,adet):
    cargo_type = gereksinim
    source_poi =konum_adi
    amount_number = adet
    rospy.wait_for_service('/karmasim_node/vehicle_load')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_load', VehicleLoad)
    res = req(uxv_name, source_poi, cargo_type, amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))

def unload_fonksiyon(hasta_adi,gereksinim):
    cargo_type = gereksinim

    target_poi =hasta_adi
    amount_number = 1
    rospy.wait_for_service('/karmasim_node/vehicle_unload')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_unload', VehicleUnload)
    res = req(uxv_name, target_poi, cargo_type, amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))

def yukselis():
    global target_lin_z_speed
    global yuksel
    target_lin_z_speed = 10
    yuksel = 1

def alcalis():
    global target_lin_z_speed
    global alcal
    target_lin_z_speed = -5
    alcal = 1

if __name__ == "__main__":
    rospy.init_node('contester_uav_node')

    sample_msg_pub = rospy.Publisher('/karmasim_node/sample_message', SampleMessage, queue_size=10)

    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]


    pub = rospy.Publisher('uav_cmd', VelCmd, queue_size=10)

    status = 0
    target_lin_x_speed = 0.0
    target_lin_y_speed = 0.0
    target_lin_z_speed = 0.0
    target_ang_x_speed = 0.0
    target_ang_y_speed = 0.0
    target_ang_z_speed = 0.0

    drone_cmd = VelCmd()
    drone_cmd.twist.linear.x = 0
    drone_cmd.twist.linear.y = 0
    drone_cmd.twist.linear.z = 0
    drone_cmd.twist.angular.x = 0
    drone_cmd.twist.angular.y = 0
    drone_cmd.twist.angular.z = 0

    uxv_name = rospy.get_param('~uxv_name', 'uav_1')
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]


    rospy.Subscriber('/karmasim_node/uxv_states', UxvStates, uxv_states_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_front', Range, radar_front_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_rear', Range, radar_rear_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_left', Range, radar_left_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_right', Range, radar_right_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_bottom', Range, radar_bottom_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/distance/dist_top', Range, radar_top_callback)
    rospy.Subscriber('/karmasim_node/points_of_interest', PointsOfInterest, yuk_states_callback)
    rospy.Subscriber('/karmasim_node/' + uxv_name + '/imu/Imu', Imu, imu_callback)
    hedef_knm=[]

    a=1
    try:
        rospy.wait_for_service('/karmasim_node/vehicle_start')
        request = rospy.ServiceProxy('/karmasim_node/vehicle_start', VehicleStart)
        result = request(uxv_name)
        time.sleep(0.5)
        print(msg)


        while 1:
            atama_modul()
            time.sleep(1)
            istasyona_mesafe()

            while len(hedef_knm) > 0:
                yuksel = 0
                alcal = 0
                x_sapma = abs(uxv_pose_.position.x - hedef_knm[0][0])
                y_sapma = abs(uxv_pose_.position.y - hedef_knm[0][1])
                z_sapma = abs(uxv_pose_.position.z - hedef_knm[0][2])
                xy_sapma = x_sapma + y_sapma
                xyz_sapma = x_sapma + y_sapma + z_sapma
                target_lin_x_speed = 0.0
                target_lin_y_speed = 0.0
                target_lin_z_speed = 0.0
                alt_radar_mesafe=uxv_radar_bottom

                if alt_radar_mesafe>1:
                    if xy_sapma > 2:
                        if x_sapma > 1:
                            if hedef_knm[0][0] > uxv_pose_.position.x:
                                arti_x_hareket()
                            else:
                                eksi_x_hareket()
                        if y_sapma > 1 and x_sapma < 3:
                            if hedef_knm[0][1] > uxv_pose_.position.y:
                                arti_y_hareket()
                            else:
                                eksi_y_hareket()
                    else:
                        if xyz_sapma > 10:
                            if hedef_knm[0][2] < uxv_pose_.position.z :
                                yuksel = 1
                            elif hedef_knm[0][2] > uxv_pose_.position.z and alt_radar_mesafe>0.75:
                                alcal = 1
                            else:
                                load_list.pop(0)
                                hedef_knm.pop(0)
                        else:
                            if load_list[0][0]==1:
                                load_fonksiyon(load_list[0][1],load_list[0][2],load_list[0][3])
                                load_list.pop(0)
                                hedef_knm.pop(0)
                            elif load_list[0][0]==2 :
                                unload_fonksiyon(load_list[0][1],load_list[0][2])
                                load_list.pop(0)
                                hedef_knm.pop(0)
                            else:
                                time.sleep(30)
                else:
                    yukselis()
                if bas_acisi_sapma > 0.1:
                    if bas_acisi > 2.7:
                        target_ang_z_speed = 0.2
                    if bas_acisi < -2.7:
                        target_ang_z_speed = -0.2
                else:
                    target_ang_z_speed=0
                if alcal == 0 and yuksel == 0:
                    target_lin_z_speed = 0

                    """elif yuksel == 1 and alcal==0:
                    target_lin_z_speed = 5
                    print("yuksel1")"""
                elif alcal == 1:
                    target_lin_z_speed = -10
                if hover == 1:
                    target_lin_x_speed = 0
                    target_lin_y_speed = 0
                    target_lin_z_speed = 0
                if uxv_radar_front<1+anlik_z_hiz/3 and target_lin_z_speed>0:
                    target_lin_z_speed=0

                if uxv_radar_front<1+anlik_x_hiz/3:
                    target_lin_x_speed=10
                if uxv_radar_rear<1+anlik_x_hiz/3:
                    target_lin_x_speed=-10
                if uxv_radar_right<1+anlik_y_hiz/3:
                    target_lin_y_speed=10
                if uxv_radar_left<1+anlik_y_hiz/3:
                    target_lin_y_speed=-10
                if kalan_yakit<6000:
                    hedef_knm=[]
                drone_cmd.twist.linear.x = target_lin_x_speed
                drone_cmd.twist.linear.y = target_lin_y_speed
                drone_cmd.twist.linear.z = -target_lin_z_speed

                drone_cmd.twist.angular.x = target_ang_x_speed
                drone_cmd.twist.angular.y = target_ang_y_speed
                drone_cmd.twist.angular.z = target_ang_z_speed

                pub.publish(drone_cmd)
                if status == 20:
                    status = 0

    except:
        print('Exception:' + e)
    finally:
        drone_cmd = VelCmd()
        drone_cmd.twist.linear.x = 0
        drone_cmd.twist.linear.y = 0
        drone_cmd.twist.linear.z = 0
        drone_cmd.twist.angular.x = 0
        drone_cmd.twist.angular.y = 0
        drone_cmd.twist.angular.z = 0
        pub.publish(drone_cmd)
        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)

