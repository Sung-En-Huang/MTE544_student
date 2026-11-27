
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 

        # TurtleBot 4 measures 342 × 339 × 351 mm (L × W × H)
        self.m_utilites=mapManipulator(filename_=self.mapName, laser_sig=0.3) # SEH originally laser_sig=0.01, changed to 0.3 to account for the robot size and to have it working in gazebo + rviz (essentially inflating the objects in the map)
        
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        # get the cell path or route from A* 
        cell_path = search(self.costMap, startPose, endPose)
        # convert each cell in the retrieved path to a list of cartesian coordinates
        Path = list(map(self.m_utilites.cell_2_position, cell_path)) # map() calls cell_2_position for each element in cell_path which is a list of tuples

        # TODO PART 5 return the path as list of [x,y]
        return Path # returns as list of tuples (x, y), will need to change to return [list(point) for point in Path] if list of lists is needed




if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
