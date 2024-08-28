#!/usr/bin/env python

import math

from easy_markers.generator import Marker, MarkerGenerator

from geometry_msgs.msg import Point, Vector3

from kalman_filter import Kalman

from people_msgs.msg import People, Person, PositionMeasurementArray

from gazebo_msgs.msg import ModelStates

import rospy


def distance(leg1, leg2):
    return math.sqrt(math.pow(leg1.x - leg2.x, 2) +
                     math.pow(leg1.y - leg2.y, 2) +
                     math.pow(leg1.z - leg2.z, 2))
def norm(leg1):
    return math.sqrt(math.pow(leg1.x, 2) +
                     math.pow(leg1.y, 2) +
                     math.pow(leg1.z, 2))

def average(leg1, leg2):
    return Point((leg1.x + leg2.x) / 2,
                 (leg1.y + leg2.y) / 2,
                 (leg1.z + leg2.z) / 2)


def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)


def subtract(v1, v2):
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)


def scale(v, s):
    v.x *= s
    v.y *= s
    v.z *= s


def printv(v):
    print('%.2f %.2f %.2f' % (v.x, v.y, v.z),)


gen = MarkerGenerator()
gen.type = Marker.ARROW
gen.ns = 'velocities'
gen.lifetime = .5


class PersonEstimate(object):
    def __init__(self, msg):
        self.pos = msg
        self.reliability = 0.1
        self.k = Kalman()

    def update(self, msg):
        last = self.pos
        self.pos = msg
        self.reliability = max(self.reliability, msg.reliability)

        ivel = subtract(self.pos.pos, last.pos)
        time = (self.pos.header.stamp - last.header.stamp).to_sec()
        scale(ivel, 1.0 / time)

        self.k.update([ivel.x, ivel.y, ivel.z])

    def age(self):
        return self.pos.header.stamp

    def get_id(self):
        return self.pos.object_id

    def velocity(self):
        k = self.k.values()
        if k is None:
            return Vector3()
        v = Vector3(k[0], k[1], k[2])
        return v

    def publish_markers(self, pub):
        gen.scale = [.1, .3, 0]
        gen.color = [1, 1, 1, 1]
        vel = self.velocity()
        m = gen.marker(points=[self.pos.pos, add(self.pos.pos, vel)])
        m.header = self.pos.header
        pub.publish(m)

    def get_person(self):
        p = Person()
        p.name = self.get_id()
        p.position = self.pos.pos
        p.velocity = self.velocity()
        p.reliability = self.reliability
        return self.pos.header.frame_id, p


class VelocityTracker(object):
    def __init__(self):
        self.people = {}
        self.TIMEOUT = rospy.Duration(rospy.get_param('~timeout', 1.0))
        self.sub = rospy.Subscriber('/people_tracker_measurements',
                                    PositionMeasurementArray,
                                    self.pm_cb)
        self.sub = rospy.Subscriber('/gazebo/model_states',
                                    ModelStates,
                                    self.gazebo_cb)
        self.mpub = rospy.Publisher('/visualization_marker',
                                    Marker,
                                    queue_size=10)
        self.ppub = rospy.Publisher('/people',
                                    People,
                                    queue_size=10)
        self.people_time = rospy.Time.now();
        self.people_num = 50
        self.people1_pos = [Vector3(-10000, -10000.0, -10000.0) for i in range(self.people_num)]
        # self.people1_pos[0] = Vector3(0.0, 0.0, 0.0)
        # self.people1_pos[1] = Vector3(0.0, 0.0, 0.0)
        # self.people1_pos[2] = Vector3(0.0, 0.0, 0.0)

        self.vel = [Vector3(0.0, 0.0, 0.0) for i in range(self.people_num)]

    def pm_cb(self, msg):
        for pm in msg.people:
            if pm.object_id in self.people:
                self.people[pm.object_id].update(pm)
            else:
                p = PersonEstimate(pm)
                self.people[pm.object_id] = p

    def gazebo_cb(self, msg):
        time = (rospy.Time.now() - self.people_time).to_sec()
        num_of_peo = 0
        if time>0.3:
            for i in range(0,len(msg.name)):
                if num_of_peo>self.people_num:
                    break
                if msg.name[i][:5] == "actor":
                    self.vel[num_of_peo] = subtract(msg.pose[i].position, self.people1_pos[num_of_peo])
                    scale(self.vel[num_of_peo], 1.0 / time)
                    if norm(self.vel[num_of_peo])>4.0:
                        self.vel[num_of_peo] = Vector3(0.0, 0.0, 0.0);                 
                    self.people1_pos[num_of_peo] = msg.pose[i].position
                    self.people_time = rospy.Time.now()
                    num_of_peo = num_of_peo+1
        # people_new_pose = [msg.pose[6].position.x, msg.pose[6].position.y, msg.pose[6].position.z]
        # print(msg.pose[6].position.x);



    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Remove People Older Than timeout param
            now = rospy.Time.now()
            for p in list(self.people.values()):
                if now - p.age() > self.TIMEOUT:
                    del self.people[p.get_id()]
            self.publish()
            rate.sleep()

    def publish(self):
        gen.counter = 0
        pl = People()
        pl.header.frame_id = "map"

        # for p in self.people.values():
        #     p.publish_markers(self.mpub)
        for i in range(self.people_num):
            if self.people1_pos[i].x <-1000:
                continue
            gen.scale = [.1, .3, 0]
            gen.color = [1, 1, 1, 1]
            vel = self.vel[i]
            m = gen.marker(points=[self.people1_pos[i], add(self.people1_pos[i], vel)])
            m.header.frame_id = "map"
            self.mpub.publish(m)

            # pl.header.frame_id = frame
            p = Person()
            p.name = str(i)
            p.position = self.people1_pos[i]
            p.velocity = self.vel[i]
            p.reliability = 1.0      
            pl.people.append(p)

        self.ppub.publish(pl)


if __name__ == '__main__':
    rospy.init_node('people_velocity_tracker')
    vt = VelocityTracker()
    vt.spin()
