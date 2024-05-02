import numpy as np
import requests
import matplotlib.pyplot as plt

class CleaningRobot:
    def __init__(self):
        self.base_url = 'http://192.168.1.23:10009'
        self.map_id = None
        self.x1 = -50
        self.y1 = -50
        self.threshold_x = -1300  # X座標の閾値
        self.threshold_y = -2500  # Y座標の閾値

    def send_request(self, url):
        req = requests.get(url)
        return req.json()

    def get_grid_map(self):
        url = f'{self.base_url}/get/cleaning_grid_map'
        return self.send_request(url)

    def get_feature_map(self):
        url = f'{self.base_url}/get/feature_map'
        return self.send_request(url)

    def get_robot_pose(self):
        url = f'{self.base_url}/get/rob_pose'
        return self.send_request(url)

    def send_clean_spot_request(self, x, y):
        if self.map_id is not None:
            url = f'{self.base_url}/set/clean_spot?map_id={self.map_id}&x1={x}&y1={y}&cleaning_parameter_set=1'
            print(url)
            requests.get(url)

    def draw_grid_map(self):
        grid_data = self.get_grid_map()
        lower_left_x = grid_data.get('lower_left_x')
        lower_left_y = grid_data.get('lower_left_y')
        size_x = grid_data.get('size_x')
        size_y = grid_data.get('size_y')
        resolution = grid_data.get('resolution')
        cleaned = grid_data.get('cleaned')

        if len(cleaned) > 0:
            map_data = [['cleaned' for _ in range(size_x)] for _ in range(size_y)]
            x, y = 0, 0

            for i in range(len(cleaned)):
                if i % 2 == 0:
                    x += cleaned[i]
                else:
                    for _ in range(cleaned[i]):
                        map_data[y][x] = 'not cleaned'
                        x += 1
                        if x == size_x:
                            x = 0
                            y += 1
                            if y == size_y:
                                break

            for i in range(size_y):
                for j in range(size_x):
                    if map_data[i][j] == 'cleaned':
                        plt.fill([lower_left_x + j * resolution, lower_left_x + (j + 1) * resolution,
                                  lower_left_x + (j + 1) * resolution, lower_left_x + j * resolution],
                                 [lower_left_y + i * resolution, lower_left_y + i * resolution,
                                  lower_left_y + (i + 1) * resolution, lower_left_y + (i + 1) * resolution],
                                 color='blue', alpha=0.5)
                        
            x = np.arange(lower_left_x, lower_left_x + (size_x + 1) * resolution, resolution)
            y = np.arange(lower_left_y, lower_left_y + (size_y + 1) * resolution, resolution)

            groups = []
            current_group = None

            for i in range(len(map_data)):
                for j in range(len(map_data[i])):
                    if map_data[i][j] == 'not cleaned':
                        if current_group is None:
                            current_group = [(i, j)]
                        else:
                            current_group.append((i, j))
                    elif current_group is not None:
                        groups.append(current_group)
                        current_group = None

            if current_group is not None:
                groups.append(current_group)

            groups = [group for group in groups if len(group) > 0]

            group_centers = []
            for group in groups:
                group_x = [x[j] for i, j in group]
                group_y = [y[i] for i, j in group]
                center_x = sum(group_x) / len(group_x)
                center_y = sum(group_y) / len(group_y)
                group_centers.append((center_x, center_y))

            colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow', 'brown', 'lime']
            for i, group in enumerate(groups):
                plt.scatter([x[j] for i, j in group], [y[i] for i, j in group], color=colors[i%10], marker='.', label=f'Group {i+1}', s=3)
            for i, center in enumerate(group_centers):
                plt.scatter(center[0], center[1], color=colors[i%10], marker='o', label='Center', s=6)
        
        plt.xticks(np.arange(lower_left_x, lower_left_x + (size_x + 1) * resolution, resolution))
        plt.yticks(np.arange(lower_left_y, lower_left_y + (size_y + 1) * resolution, resolution))

    def draw_feature_map(self):
        map_data = self.get_feature_map()
        map_id = map_data.get('map').get('map_id')
        map_lines = map_data.get('map').get('lines')
        docking_pose = map_data.get('map').get('docking_pose')
        for line in map_lines:
            x1 = line.get('x1')
            y1 = line.get('y1')
            x2 = line.get('x2')
            y2 = line.get('y2')
            plt.plot([x1, x2], [y1, y2], color='gray')
        plt.plot([docking_pose.get('x')], [docking_pose.get('y')], marker="D", markersize=6)  # ドック
        plt.title("map_id : {}".format(map_id))

        polygons_data = self.send_request(f'{self.base_url}/get/n_n_polygons')
        polygons = polygons_data.get('map').get('polygons')

        for polygon in polygons:
            seg = polygon.get('segments')
            for segment in seg:
                x1 = segment.get('x1')
                y1 = segment.get('y1')
                x2 = segment.get('x2')
                y2 = segment.get('y2')
                plt.plot([x1, x2], [y1, y2], color='gray')

        self.map_id = map_id

    def draw_robot_position(self):
        pose_data = self.get_robot_pose()
        x1 = pose_data.get('x1')
        y1 = pose_data.get('y1')
        plt.plot([x1], [y1], marker="*", markersize=12)  # イマココ
        self.x1 = x1
        self.y1 = y1

    def check_position_exceed_threshold(self):
        plt.axvline(x=self.threshold_x, color='red', linestyle='--', label='threshold_x')
        plt.axhline(y=self.threshold_y, color='green', linestyle='--', label='threshold_y')

        if self.x1 < self.threshold_x or self.y1 < self.threshold_y:
            self.send_clean_spot_request(0,0)

    def main_loop(self):
        plt.clf()  # グラフをクリアして新しいフレームを描画する準備をする
        self.draw_grid_map()
        self.draw_feature_map()
        self.draw_robot_position()
        self.check_position_exceed_threshold()
        plt.grid()
        plt.axis('equal')
        plt.savefig('result.png')
        plt.pause(1)  # 1秒待機する

if __name__ == '__main__':
    cleaning_robot = CleaningRobot()
    while True:
        cleaning_robot.main_loop()
