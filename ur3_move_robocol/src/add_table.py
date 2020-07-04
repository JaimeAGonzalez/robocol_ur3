#!/usr/bin/env python2
import rospy
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest

class ObjectServer(object):
	def __init__(self):
		self.scene = PlanningSceneInterface()
		self.robot = RobotCommander()
		self.p = PoseStamped()
		self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")

	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo("Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)
			# check if 'part' is in the answer
			for collision_obj in gps_resp.scene.world.collision_objects:
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			else:
				rospy.sleep(1.0)
		rospy.loginfo("'" + object_name + "'' is in scene!")


	def make_object(self,object_name,x,y,z,sx,sy,sz):
		rospy.loginfo("Removing any previous 'part' object")
		self.scene.remove_attached_object("arm_tool_link")
		self.scene.remove_world_object(object_name)
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")
		rospy.loginfo("Making "+object_name+"...")
		self.p.header.frame_id = self.robot.get_planning_frame()
		self.p.pose.position.x = x
		self.p.pose.position.y = y
		self.p.pose.position.z = z
		self.scene.add_box(object_name, self.p, (sx,sy,sz))
		self.wait_for_planning_scene_object(object_name)

if __name__ == '__main__':
	rospy.init_node('table_node',anonymous=True)
	#moveit_commander.roscpp_initialize(sys.argv)
	objects = ObjectServer()
	objects.make_object("table",0.0,0.0,-0.25,0.5,0.5,0.5)
	objects.make_object("panel",0.0,0.4,0.25,0.5,0.1,0.5)
	rospy.spin()
