import json
import matplotlib.pyplot as plt
import numpy as np
import sys

scenes = ['scene1.json', 'scene2.json', 'scene3.json', 'scene4.json', 'scene5.json']
colors = ['blue', 'orange', 'green', 'purple']

def loadScene(filename):
    with open(f'../src/inputs/experiment/{filename}', 'r') as f:
        return json.load(f)

def plotObstacles(obstacles):
    for obstacle in obstacles:
        points = np.array([[p['x'], p['y']] for p in obstacle['points']])
        polygon = plt.Polygon(points, closed=True, fill=True, edgecolor='black', facecolor='gray', alpha=0.5)
        plt.gca().add_patch(polygon)
        yield points

def plotRobot(robot, angles, color, alpha):
    x, y = robot['pinnedPosition']['x'], robot['pinnedPosition']['y']
    joint_length = robot['jointLength']
    angles_rad = np.radians(angles)
    
    joints = [(x, y)]
    current_angle = 0
    
    for angle in angles_rad:
        current_angle += angle
        x += joint_length * np.cos(current_angle)
        y += joint_length * np.sin(current_angle)
        joints.append((x, y))
    
    joints = np.array(joints)
    plt.plot(joints[:, 0], joints[:, 1], color=color, alpha=alpha)
    plt.scatter(joints[:, 0], joints[:, 1], color=color, alpha=alpha)
    return joints


def main():
    for _scene in scenes:
        scene = loadScene(_scene)
        plt.figure(figsize=(6, 6))
        plt.gca().set_aspect('equal', adjustable='box')
        
        plotObstacles(scene['obstacles'])
        allPoints = []

        # Plot obstacles and collect all points
        for points in plotObstacles(scene['obstacles']):
            allPoints.extend(points)
        
        # Plot robots and collect all points
        for idx, robot in enumerate(scene['robots']):
            startPoints = plotRobot(robot, robot['startAngles'], colors[idx], 0.2)
            goalPoints = plotRobot(robot, robot['goalAngles'], colors[idx], 0.8)
            allPoints.extend(startPoints)
            allPoints.extend(goalPoints)
        
        allPoints = np.array(allPoints)
        
        # Calculate the bounds with some padding
        min_x, max_x = allPoints[:, 0].min(), allPoints[:, 0].max()
        min_y, max_y = allPoints[:, 1].min(), allPoints[:, 1].max()
        padding = 1
        plt.xlim(min_x - padding, max_x + padding)
        plt.ylim(min_y - padding, max_y + padding)
        
        plt.xticks([])
        plt.yticks([])

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title(_scene)
        plt.grid(False)
        plt.savefig(f'{_scene}.png')
        plt.clf()


if __name__ == "__main__":
    rc = 1
    try:
        main()
        rc = 0
    except Exception as e:
        print('Error: %s' % e, file=sys.stderr)
    sys.exit(rc)