import random
import unittest
import os

import plane_finder


class TestVectorFunctions(unittest.TestCase):

    def setUp(self):
        self.v1 = [0,0,1]
        self.v2 = [0,1,0]
        self.plane1 = [0,0,1,0]
        self.p1 = [0,0,10]
        self.p2 = [0,10,0]
        self.p3 = [10,0,0]
        self.seq = range(10)

    def test_cross(self):
        self.assertEqual(plane_finder.cross(self.v1,self.v2), [-1,0,0])

    def test_distance_1(self):
        self.assertTrue(plane_finder.compute_distance(self.p1,self.plane1) == 10)

    def test_distance_2(self):
        self.assertTrue(plane_finder.compute_distance(self.p2,self.plane1) == 0)

    def test_dot(self):
        self.assertTrue(plane_finder.dot([1,2,3],[3,2,1]),[3,4,3])

    def test_compute_plane(self):
        v = plane_finder.compute_plane([self.p1,self.p2,self.p3])
        self.assertEqual(v,[-0.5773502691896257, -0.5773502691896257, -0.5773502691896257, 5.773502691896257])


class TestPCDReadingWriting(unittest.TestCase):

    def setUp(self):
        for fn in ['test.pcd','testout.pcd']:
            f = open(fn,'w')
            self.header = ['# .PCD v.7 - Point Cloud Data file format\n', 'VERSION .7\n', 'FIELDS x y z i\n', 'SIZE 4 4 4 4\n', 'TYPE F F F F\n', 'COUNT 1 1 1 1\n', 'WIDTH 1025349\n', 'HEIGHT 1\n', 'VIEWPOINT 0 0 0 1 0 0 0)\n', 'POINTS 1025349\n', 'DATA ascii\n']
            self.contents = ['474203.830000 4423050.110000 10.84 106\n', '474203.880000 4423050.090000 -1.82 120\n', '474204.040000 4423050.120000 6.94 58\n']
            f.writelines(self.header)
            f.writelines(self.contents)
            f.close()

    def test_read(self):
        (header,contents) = plane_finder.read_pcd('test.pcd')
        self.assertEqual(header,self.header)
        for i,item in enumerate(self.contents):
            self.assertEqual(contents[i],[float(x) for x in item.strip().split(' ')])

    def test_write(self):
        (header,contents) = plane_finder.read_pcd('test.pcd')
        contents = contents[0:1]
        plane_finder.write_pcd('testout.pcd',header,contents)
        (header2,contents2) = plane_finder.read_pcd('testout.pcd')
        headertrue = ['# .PCD v.7 - Point Cloud Data file format\n', 'VERSION .7\n', 'FIELDS x y z i\n', 'SIZE 4 4 4 4\n', 'TYPE F F F F\n', 'COUNT 1 1 1 1\n', 'WIDTH 1025349\n', 'HEIGHT 1\n', 'VIEWPOINT 0 0 0 1 0 0 0)\n', 'POINTS 1\n', 'DATA ascii\n']
        self.assertEqual(contents,contents2)
        self.assertEqual(headertrue,header2)

    def tearDown(self):
        for fn in ['test.pcd','testout.pcd']:
            os.remove(fn)

class TestFindPlane(unittest.TestCase):

    def setUp(self):
        self.point_list = [(0,random.random(), random.random()) for x in range(1000)]
        self.point_list.extend([(random.random(),random.random(),random.random()) for x in range(250)])

    def test_find_plane(self):
        q = plane_finder.plane_fit(self.point_list)
        s = abs(plane_finder.dot(q[0:3],[1,0,0]))
        self.assertTrue(s > 0.95,'Ransac did not converge')



if __name__ == '__main__':
    unittest.main()
