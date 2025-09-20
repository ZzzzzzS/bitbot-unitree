#!/usr/bin/python3
# Reference: https://github.com/unitreerobotics/unitree_mujoco/blob/main/terrain_tool/terrain_generator.py
# Dknt 2025.8

import xml.etree.ElementTree as xml_et
import numpy as np

ROOT_PATH = "/home/dknt/Project/bitbot_gz_ws/src/bitbot_gz/world/"
INPUT_SCENE_PATH = ROOT_PATH + "empty.sdf"
OUTPUT_SCENE_PATH = ROOT_PATH + "terrain.sdf"


# zyx euler angle to quaternion
def euler_to_quat(roll, pitch, yaw):
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float64,
    )


# zyx euler angle to rotation matrix
def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ],
        dtype=np.float64,
    )

    rot_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ],
        dtype=np.float64,
    )
    rot_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


# 2d rotate
def rot2d(x, y, yaw):
    nx = x * np.cos(yaw) - y * np.sin(yaw)
    ny = x * np.sin(yaw) + y * np.cos(yaw)
    return nx, ny


# 3d rotate
def rot3d(pos, euler):
    R = euler_to_rot(euler[0], euler[1], euler[2])
    return R @ pos


def list_to_str(vec):
    return " ".join(str(s) for s in vec)


class TerrainGenerator:

    def __init__(self) -> None:
        self.scene = xml_et.parse(INPUT_SCENE_PATH)
        self.root = self.scene.getroot()
        self.world = self.root.find("world")
        # self.model = self.world.find("model")
        self.model = self.world.find("model")
        self.box_index = 0

    # Add Box to scene
    def AddBox(
        self, position=[1.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0], size=[0.1, 0.1, 0.1]
    ):
        link = xml_et.SubElement(self.model, "link")
        link.attrib["name"] = "link" + str(self.box_index)
        pose = xml_et.SubElement(link, "pose")
        pose.text = list_to_str(position + euler)

        collision = xml_et.SubElement(link, "collision")
        collision.attrib["name"] = "collision" + str(self.box_index)
        collision_geo = xml_et.SubElement(collision, "geometry")
        collision_box = xml_et.SubElement(collision_geo, "box")
        collision_size = xml_et.SubElement(collision_box, "size")
        collision_size.text = list_to_str(size)

        visual = xml_et.SubElement(link, "visual")
        visual.attrib["name"] = "visual" + str(self.box_index)
        visual_geo = xml_et.SubElement(visual, "geometry")
        visual_box = xml_et.SubElement(visual_geo, "box")
        visual_size = xml_et.SubElement(visual_box, "size")
        visual_size.text = list_to_str(size)
        material = xml_et.SubElement(visual, "material")
        ambient = xml_et.SubElement(material, "ambient")
        diffuse = xml_et.SubElement(material, "diffuse")
        specular = xml_et.SubElement(material, "specular")

        r = np.random.uniform(0, 1)
        g = np.random.uniform(0, 1 - r)
        b = 1 - r - g

        ambient.text = f"{r} {g} {b} 0.95"
        diffuse.text = f"{r} {g} {b} 0.95"
        specular.text = f"{r} {g} {b} 0.95"
        self.box_index += 1

    def AddStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=0.0,
        width=0.2,
        height=0.15,
        length=1.5,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, height],
            )

    def AddSuspendStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=1.0,
        width=0.2,
        height=0.15,
        length=1.5,
        gap=0.1,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, abs(height - gap)],
            )

    def AddRoughGround(
        self,
        init_pos=[1.0, 0.0, 0.0],
        euler=[0.0, -0.0, 0.0],
        nums=[10, 10],
        box_size=[0.5, 0.5, 0.5],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.2, 0.2],
        box_size_rand=[0.05, 0.05, 0.05],
        box_euler_rand=[0.2, 0.2, 0.2],
        separation_rand=[0.05, 0.05],
    ):

        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_separation = np.array(separation) + np.array(
            separation_rand
        ) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_separation[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(
                    box_size_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(box_euler) + np.array(
                    box_euler_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_separation = np.array(separation) + np.array(
                    separation_rand
                ) * np.random.uniform(-1.0, 1.0, 2)

                local_pos[1] += new_separation[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.AddBox(
                    position=pos.tolist(),
                    euler=new_box_euler.tolist(),
                    size=new_box_size.tolist(),
                )

    def Save(self):
        self.scene.write(OUTPUT_SCENE_PATH)


def DiscreteUneven(tg: TerrainGenerator):
    tg.AddRoughGround(
        init_pos=[1.5, -3.0, 0.005],
        euler=[0, 0, 0.0],
        nums=[15, 15],
        box_size=[0.5, 0.5, 0.3],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.5, 0.5],
        box_size_rand=[0.2, 0.2, 0.01],
        # box_euler_rand=[0.07, 0.07, 0.3],
        box_euler_rand=[0.19, 0.19, 0.3],
        separation_rand=[0.05, 0.05],
    )


def Slope(tg: TerrainGenerator):
    tg.AddBox(position=[-1.0, 0.0, 0.0], euler=[0.0, 0.20, 0.0], size=[10.0, 5.0, 0.03])


def Stairs(tg: TerrainGenerator, init_pos=np.array([1.0, 2.0, 0.0])):
    height = 0.3
    width = 1.0
    stair_nums = 5

    init_pos_up = init_pos + np.array([0.0, 0.0, 0.0])
    init_pos_down = init_pos + np.array([width * (stair_nums * 2 + 1), 0.0, 0.0])

    tg.AddStairs(
        init_pos=init_pos_up.tolist(),
        yaw=0.0,
        width=width,
        height=height,
        length=2.0,
        stair_nums=stair_nums,
    )
    tg.AddStairs(
        init_pos=init_pos_down.tolist(),
        yaw=3.14,
        width=width,
        height=height,
        length=2.0,
        stair_nums=stair_nums,
    )


def Gap(tg: TerrainGenerator, init_pos=np.array([1.0, -2.0, 0.0])):
    bias_x = 3.0
    platform_l = 1.0
    gap_size = [0.6, 0.6, 0.6, 0.6]
    height = 0.5

    slope_pos = init_pos + np.array([bias_x - 2.46 / 2, 0.0, height / 2])
    tg.AddBox(
        position=slope_pos.tolist(),
        euler=[0.0, -0.20, 0.0],
        size=[2.51, 2.0, 0.005],
    )

    box_pos = init_pos + np.array([bias_x + platform_l / 2.0, 0.0, height / 2])
    tg.AddBox(
        position=box_pos.tolist(),
        euler=[0.0, 0.0, 0.0],
        size=[platform_l, 2.0, height],
    )

    for gap in gap_size:
        box_pos = box_pos + np.array([platform_l + gap, 0.0, 0.0])
        tg.AddBox(
            position=box_pos.tolist(),
            euler=[0.0, 0.0, 0.0],
            size=[platform_l, 2.0, height],
        )


if __name__ == "__main__":
    tg = TerrainGenerator()

    Gap(tg)
    Stairs(tg)

    tg.Save()
