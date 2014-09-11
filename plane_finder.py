import os
import sys
import math
import random
import itertools

"""
Plane finder program. Receives two point clouds and input and an output. Finds
the largest upright plane and cleans it up of other junk.
 
Usage:
python plane_finder.py input.pcd output.pcd
"""
 
HEADER_FIELDS = ['#', 'VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']
 
 
def isplit(pred, iterable):
    a, b = itertools.tee(iterable, 2)
    return (itertools.takewhile(pred, a), itertools.dropwhile(pred, b))
 
 
def is_field(line):
    try:
        found = (field for field in HEADER_FIELDS if line.startswith(field)).next()
        return True
    except StopIteration:
        return False
 
 
def split_pcf(f):
    return isplit(is_field, f)
 
def read_pcd(filename):
    "Read the PCD file, return the header and the data"
    with open(filename) as f:
        (h, l) = split_pcf(f)
        point_cloud = []
        for item in l:
            item = item.strip().split()
            parts = map(float, item)
            point_cloud.append(parts[:4])
    return (list(h), point_cloud)

def write_pcd(filename,header,pcl):
    "Write a PCD file, adjust the header with the number of points in the point cloud pcl"
    f = open(filename,'w')
    for item in header:
        if 'POINTS' in item:
            f.write('POINTS %d\n'%len(pcl))
        else:
            f.write(item.strip()+'\n')
    for item in pcl:
        f.write(' '.join([str(x) for x in item])+'\n')
    f.close()

def cross(a, b):
    "cross product of two vectors. The vector will be perpendicular to a and b"
    c = [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]
    return c

def dot(a,b):
    "dot product of two vectors"
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def compute_plane(points):
    """
    This function receives three points and returns a [a,b,c,d] representation
    of a plane, where ax+by+cz+d = 0 and (a,b,c) are the normalized (unit
    length) normal
    """
    va = [points[0][0] - points[1][0],points[0][1] - points[1][1],points[0][2] - points[1][2]]
    vb = [points[0][0] - points[2][0],points[0][1] - points[2][1],points[0][2] - points[2][2]]
    normal = cross(va,vb)
    na = normal[0]/math.sqrt(normal[0]**2+normal[1]**2+normal[2]**2)
    nb = normal[1]/math.sqrt(normal[0]**2+normal[1]**2+normal[2]**2)
    nc = normal[2]/math.sqrt(normal[0]**2+normal[1]**2+normal[2]**2)
    normal = [na,nb,nc]
    return [normal[0],normal[1],normal[2],-normal[0]*points[0][0]-normal[1]*points[0][1]-normal[2]*points[0][2]]

def compute_distance(point,plane):
    """
    This function receives a point and a plane and computes the point to plane
    distance
    """
    (a,b,c,d) = plane
    num = abs(a*point[0]+b*point[1]+c*point[2]+d)
    denom = math.sqrt(a**2+b**2+c**2)
    return num/denom

def plane_fit(sample):
    max_covered = -1 
    for i in range(500):
        q = random.sample(sample,3)
        plane = compute_plane(q)
        covered = 0
        for item in sample:
            d = compute_distance(item,plane) 
            if d < 0.1:
                covered = covered + 1
        if covered>max_covered:
            max_covered = covered
            best_plane = plane

    #showpoints(sample)
    return best_plane

def showpoints(sample1):
    #This is for debugging purposes, and left here for the record
    from matplotlib import pyplot
    import pylab
    from mpl_toolkits.mplot3d import Axes3D
    x_vals = []
    y_vals = []
    z_vals = []
    c_vals = []
    for item in sample1:
        x_vals.append(item[0])
        y_vals.append(item[1])
        z_vals.append(item[2])
        c_vals.append(item[3])

    fig = pylab.figure()
    ax = Axes3D(fig)
    ax.scatter(x_vals, y_vals, z_vals,c=c_vals,marker='+')
    pyplot.show()

#This function yields the point in the range [start,stop) every step.
def drange(start, stop, step):
     r = start
     while r < stop:
     	yield r
     	r += step

def filter_frontal_plane(points,plane):
    """
    This function will try to build a 2d histogram to filter out parts that are
    not part of the structure of the house. It will build a 2d histogram along
    2 dimensions up (the second dimension) and left-right (which is the cross
    product of normal of the plane and up.
    """

    #compute the l-r direction
    lr = cross(plane[0:3],[0,1,0])
    print 'l-r orientation:'
    print lr
    mn0 = float('Inf')
    mx0 = -float('Inf')
    prj = [item[0]*lr[0]+item[1]*lr[1]+item[2]*lr[2] for item in points]
    mx0 = max(prj)
    mn0 = min(prj)
    xrng = mx0-mn0

    item1 = [item[1] for item in points]
    mn1 = min(item1) 
    mx1 = max(item1) 
    yrng = mx1-mn1

    tot = 0
    d = {}
    for i,x in enumerate(drange(mn0,mx0,xrng/10)):
        for j,y in enumerate(drange(mn1,mx1,yrng/10)):
            cnt = count_points(prj,points,x,xrng/10,y,yrng/10)
            d[(i,j)] = cnt
            tot = tot + cnt
    points_out = []        
    print '2d histogram:'
    for i,x in enumerate(drange(mn0,mx0,xrng/10)):
        for j,y in enumerate(drange(mn1,mx1,yrng/10)):
            if d[(i,j)]/float(tot) > 0.012:
                currpoints = get_points(prj,points,x,xrng/10,y,yrng/10)
                points_out.extend(currpoints)
                print 1,
            else:
                print 0,
        print 
    return points_out

def count_points(prj,points,x,xrng,y,yrng):
    """
    Given a lr direction values and corresponding points, this counts the
    points that fall between x and x+xrng (for the lr direction) and y and
    y+yrng for the height
    """
    count = 0
    for prj,item in zip(prj,points):
        if prj>=x and prj<x+xrng and item[1]>=y and item[1]<y+yrng:
            count = count + 1
    return count

def get_points(prj,points,x,xrng,y,yrng):
    """
    Given a lr direction values and corresponding points, this returs a list
    with the points that fall between x and x+xrng (for the lr direction) and
    y and y+yrng for the height
    """
    points_out = []
    for prj,item in zip(prj,points):
        if prj>=x and prj<x+xrng and item[1]>=y and item[1]<y+yrng:
            points_out.append(item)
    return points_out

def main(args):

    if len(args)!=2:
        print 'Usage: python plane_finder.py input.pcd output.pcd'
        sys.exit(0)

    #Load the pcl, and hold on to the header
    (header,pcl) = read_pcd(args[0])

    #Robustly find planes that onto which a lot of points fall.
    curpcl = pcl
    for tries in range(100):
        covered = 0
        plane = plane_fit(random.sample(curpcl,5000))
        curpcl2 = []
        planef = []
        for item in curpcl:
            d =compute_distance(item,plane) 
            if d < 0.1:
                covered = covered + 1
                planef.append(item)
            else:
                curpcl2.append(item)
        curpcl = curpcl2[:]

        #Check to see that the main wall is more or less upright,
        #that is, check that the unit length normal dot product with the normal
        #vector of the ground floor is more or less zero
        n = plane[0:3] 
        print 'checking:',plane
        if abs(dot(n,[0,0,1])) < 0.15:
            #We're done when the plane we found has:
            #(1) a lot of points and
            #(2) is mostly upright
            print 'main wall:',plane,covered
            planef_filtered = filter_frontal_plane(planef,plane)
            write_pcd(args[1],header,planef_filtered)
            print
            print 'Output file: %s was generated successfully'%args[1]
            print 
            #showpoints(planef_filtered)
            break
        
        
if __name__ == '__main__':
    main(sys.argv[1:])
