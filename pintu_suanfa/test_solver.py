"""有意义的几何回归：旋转散放、三角切割、凹片、无解、坐标变换、越界。"""
import unittest
import contextlib
import io
import numpy as np
from puzzle_solver import Piece, Config, solve, overlap, Pose, make_tasks, rotation, centroid, demo


class GeometryTests(unittest.TestCase):
    def test_four_piece_demo(self):
        with contextlib.redirect_stdout(io.StringIO()):
            pieces, result, output = demo()
        self.assertEqual(len(output['tasks']),4)
        s=result['solutions'][0]
        self.assertAlmostEqual(s['width_mm']*s['height_mm'],6000,places=5)
        for i in range(4):
            for j in range(i+1,4):
                self.assertLess(overlap(pieces[i],s['poses'][i],pieces[j],s['poses'][j]),1e-6)

    def test_triangles_random_pose(self):
        polys = [np.array(p, float) for p in [[(0,0),(100,0),(0,60)], [(100,0),(100,60),(0,60)]]]
        pieces = []
        for i,p in enumerate(polys):
            q = p @ rotation([0.41,-0.72][i]).T + [i*120,25]
            pieces.append(Piece(str(i),q,centroid(q)))
        result = solve(pieces,Config(max_seconds=5,max_solutions=4))
        self.assertTrue(result['solutions'])
        s = result['solutions'][0]
        self.assertAlmostEqual(s['width_mm']*s['height_mm'],6000,places=5)
        self.assertLess(overlap(pieces[0],s['poses'][0],pieces[1],s['poses'][1]),1e-6)
        tasks = make_tasks(pieces,s,[30,180])
        for p,t in zip(pieces,tasks['tasks']):
            np.testing.assert_allclose(np.array(t['source_to_target_R']) @ p.pick_mm+t['source_to_target_t_mm'],t['place_xy_mm'])
        with self.assertRaises(ValueError):
            make_tasks(pieces,s,[200,290])

    def test_concave_and_overlap(self):
        p = Piece('L',[(0,0),(100,0),(100,30),(40,30),(0,60)],[20,20])
        identity = Pose(0,np.zeros(2))
        self.assertAlmostEqual(overlap(p,identity,p,identity),3600,places=5)

    def test_no_rectangle(self):
        p=Piece('triangle',[(0,0),(100,0),(0,60)],[10,10])
        self.assertEqual(solve([p])['status'],'no_solution')

    def test_single_rectangle(self):
        p=Piece('rectangle',[(0,0),(100,0),(100,60),(0,60)],[50,30])
        self.assertTrue(solve([p])['solutions'])


if __name__ == '__main__':
    unittest.main()
