import numpy as np

class IkSolver:
    def __init__(self, a1, a2, d1, d4):
        self.a1 = a1
        self.a2 = a2
        self.d1 = d1
        self.d4 = d4
        
    # Inverse kinematics solver  
    def ikSolve(self, Ox, Oy, Oz, alpha):
        d3 = self.d1 - self.d4 - Oz
        
        D = (Ox**2 + Oy**2 - self.a1**2 - self.a2**2)/(2*self.a1*self.a2)
        th2 = np.arctan2(np.sqrt(1 - D**2), D)
        
        th1 = np.arctan2(Oy, Ox) - np.arctan2(self.a2*np.sin(th2), self.a1 + self.a2*np.cos(th2))
        
        th4 = th1 + th2 -alpha
        
        return th1, th2, d3, th4
    
if __name__=="__main__":
   # Create an object of the Scara class
   my_scara = iKSolver(0.5, 0.5, 1.0, 0.2)
   
   # Inverse kinematics
   th1, th2, d3, th4 = my_scara.ikSolve(0.7, 0.3, 0.1, np.pi/5)
   
   # Printing results
   print('Joint space: ', th1, th2, d3, th4)
   
   
   
   
   
   
   
   
   
        
        
