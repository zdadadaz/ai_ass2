import unittest
from EST import EST 
from test_robot import test_robot
from visualiser import Visualiser
from tester import tester_main
from interpolation import Interpolation
from util import write_sampling_config

class test_EST(unittest.TestCase):
    def test3g(self):
        files = ['./testcases/4g1_m1.txt']
        for i in range(len(files)):
            fileName = files[i].split('/')[-1]
            sol = './out/'+ fileName[:-4] + '_output.txt'
            prm = EST(files[i])
            prm.run_EST(sol)
            aa = test_robot(prm)
            qq = aa.load_output(sol)
            strlist = []
            for j in qq:
                strlist.append(str(j))
            gginterpolat = Interpolation(strlist)
            interpolation_path = sol[:-4] +'_ip.txt'
            gg = gginterpolat.run_Interpolate()
            write_sampling_config(interpolation_path,prm.num_segments,gg)
            result = tester_main(files[i],interpolation_path)
            if (result == 0):
                print(files[i] + ' success')
            else:
                print(files[i] + ' fails')
            self.assertEqual(result,0)

            # vis = Visualiser(prm, qq)
            

if __name__ == '__main__':
    unittest.main()