import unittest
from PRM import PRM 
from test_robot import test_robot
from visualiser import Visualiser
from tester import tester_main
from interpolation import Interpolation
from util import write_sampling_config

class test_PRM(unittest.TestCase):
    def test3g(self):
        
        # files = ['./testcases/3g3_m1.txt','./testcases/4g3_m2.txt']
        
        # colide
        # check collision more times in long distance one
        files = ['./testcases/4g1_m2.txt'] 

        # no result
        # files = ['./testcases/4g1_m2.txt','./testcases/5g3_m1.txt']
        
        # use global is enough, or the last with local
        # files = ['./testcases/3g2_m1.txt','./testcases/3g2_m2.txt', './testcases/3g3_m1.txt']
        # files = ['./testcases/3g1_m0.txt','./testcases/3g1_m1.txt','./testcases/3g1_m2.txt','./testcases/4g1_m1.txt', './testcases/4g1_m2.txt']
        for i in range(len(files)):
            fileName = files[i].split('/')[-1]
            sol = './out/'+ fileName[:-4] + '_output.txt'
            prm = PRM(files[i])
            flagPRM,ee1flag = prm.run_PRM(sol)
            # flagPRM=True
            if flagPRM:
                aa = test_robot(prm)
                qq = aa.load_output(sol)
                strlist = []
                for j in qq:
                    strlist.append(str(j))
                # ee1flag=[]
                gginterpolat = Interpolation(strlist,ee1flag)
                interpolation_path = sol[:-4] +'_ip.txt'
                print("Run interpolation")
                gg = gginterpolat.run_Interpolate()
                write_sampling_config(interpolation_path,prm.num_segments,gg)
                result = tester_main(files[i],interpolation_path)
                if (result == 0):
                    print(files[i] + ' success')
                else:
                    print(files[i] + ' fails')
            else:
                print(files[i] + ' fails')
                result = 1
            self.assertEqual(result,0)

            # vis = Visualiser(prm, qq)
            

if __name__ == '__main__':
    unittest.main()