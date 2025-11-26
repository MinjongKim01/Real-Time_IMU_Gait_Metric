import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import numpy as np

class RealTimeVisualizer:
    def __init__(self, sensor_roles=["left", "right"]):
        self.sensor_roles = sensor_roles
        self.colors = {"left": "blue", "right": "red"}
        self.fig = None
        
        # 최적화: 상수 정의
        self.MAX_VLINES_POOL = 20  # vline 객체 풀 최대 크기
        self.ANIMATION_INTERVAL = 100  # ms
        
        self.ax_traj_l = None 
        self.ax_traj_r = None
        self.ax_gyro_l = None
        self.ax_gyro_r = None
        self.ax_acc_l = None
        self.ax_acc_r = None
        
        self.traj_lines = {}
        self.traj_hs_scatters = {}
        self.traj_to_scatters = {}
        self.traj_stride_annotations = {}  # stride length 표시용 annotation 저장
        
        self.gyro_lines = {}
        # 최적화: vline 객체 풀 (재사용)
        self.gyro_hs_vlines_pool = {role: [] for role in sensor_roles}
        self.gyro_to_vlines_pool = {role: [] for role in sensor_roles}
        self.gyro_hs_vlines_active = {role: 0 for role in sensor_roles}  # 활성 vline 수
        self.gyro_to_vlines_active = {role: 0 for role in sensor_roles}
        
        self.acc_lines = {role: {} for role in sensor_roles} 
        
        self._is_closed = False
        self.reset_button = None
        
        self.analyzer_ref = None
        self.xdpc_handler_ref = None
    
    def _create_vline_pool(self, ax, pool_list, color, linestyle='--'):
        """vline 객체 풀을 미리 생성 (재사용용)"""
        for _ in range(self.MAX_VLINES_POOL):
            line = ax.axvline(0, color=color, linestyle=linestyle, linewidth=1, visible=False)
            pool_list.append(line)
    
    def _update_vlines(self, pool_list, active_count_dict, role, time_values):
        """vline 객체를 재사용하여 업데이트 (remove/create 없음)"""
        n_events = len(time_values)
        
        # 풀 크기가 부족하면 확장
        while len(pool_list) < n_events:
            ax = self.ax_gyro_l if role == "left" else self.ax_gyro_r
            color = 'red' if pool_list == self.gyro_hs_vlines_pool[role] else 'blue'
            line = ax.axvline(0, color=color, linestyle='--', linewidth=1, visible=False)
            pool_list.append(line)
        
        # 필요한 만큼 활성화 및 위치 업데이트
        for i, t in enumerate(time_values):
            pool_list[i].set_xdata([t, t])
            pool_list[i].set_visible(True)
        
        # 나머지는 비활성화
        for i in range(n_events, len(pool_list)):
            pool_list[i].set_visible(False)
        
        active_count_dict[role] = n_events
        
    def start(self, analyzer, xdpc_handler=None):
        self.analyzer_ref = analyzer
        self.xdpc_handler_ref = xdpc_handler

        self.fig = plt.figure(figsize=(18, 10))
        
        gs = self.fig.add_gridspec(4, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1, 1, 1])
        plt.subplots_adjust(bottom=0.15, top=0.95, left=0.05, right=0.95, hspace=0.4, wspace=0.3) 

        self.ax_traj_l = self.fig.add_subplot(gs[:, 1]) 
        self.ax_traj_r = self.fig.add_subplot(gs[:, 2])
        
        title_fontsize = 14
        label_fontsize = 12
        tick_fontsize = 10
        legend_fontsize = 10
         
        for ax, title in [(self.ax_traj_l, "Left Foot Analysis"), (self.ax_traj_r, "Right Foot Analysis")]:
            ax.set_title(f"{title}", fontsize=title_fontsize) 
            ax.set_xlabel("X (m)", fontsize=label_fontsize)
            ax.set_ylabel("Y (m)", fontsize=label_fontsize)
            ax.tick_params(labelsize=tick_fontsize)
            ax.set_aspect('auto')  # 세로축 스케일 확대를 위해 auto로 변경
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.set_xlim(-4, 4) 
            ax.set_ylim(-2, 6)  # y축 범위 변경: -7~7 -> -2~6 

        self.ax_gyro_l = self.fig.add_subplot(gs[0, 0])
        self.ax_gyro_r = self.fig.add_subplot(gs[1, 0])
        self.ax_acc_l = self.fig.add_subplot(gs[2, 0])
        self.ax_acc_r = self.fig.add_subplot(gs[3, 0])
        
        self.ax_gyro_l.set_title("Left Foot z-axis Gyro", fontsize=title_fontsize)
        self.ax_gyro_l.set_ylabel("rad/s", fontsize=label_fontsize)
        self.ax_gyro_l.set_ylim(-500, 500)
        self.ax_gyro_r.set_title("Right Foot z-axis Gyro", fontsize=title_fontsize)
        self.ax_gyro_r.set_ylabel("rad/s", fontsize=label_fontsize)
        self.ax_gyro_r.set_ylim(-500, 500)
        self.ax_acc_l.set_title("Left Foot Free Acc", fontsize=title_fontsize) 
        self.ax_acc_l.set_ylabel("m/s^2", fontsize=label_fontsize)
        self.ax_acc_l.set_ylim(-30, 30)
        self.ax_acc_r.set_title("Right Foot Free Acc", fontsize=title_fontsize)
        self.ax_acc_r.set_ylabel("m/s^2", fontsize=label_fontsize)
        self.ax_acc_r.set_ylim(-30, 30)
        self.ax_acc_r.set_xlabel("Time (s)", fontsize=label_fontsize)

        for role in self.sensor_roles:
            traj_color = self.colors.get(role, "gray")
            
            ax_traj = self.ax_traj_l if role == "left" else self.ax_traj_r
            self.traj_lines[role], = ax_traj.plot([], [], lw=2, label="trajectory", color=traj_color)
            self.traj_hs_scatters[role] = ax_traj.scatter([], [], marker='x', s=100, label="HS", color=traj_color)
            self.traj_to_scatters[role] = ax_traj.scatter([], [], marker='o', s=100, label="TO", facecolors='none', edgecolors=traj_color)
            ax_traj.legend(fontsize=legend_fontsize)
            
            # stride length annotation 초기화
            self.traj_stride_annotations[role] = []
            
            ax_gyro = self.ax_gyro_l if role == "left" else self.ax_gyro_r
            self.gyro_lines[role], = ax_gyro.plot([], [], lw=1.5, label="z-axis Gyro", color='black', alpha=0.8)
            
            ax_acc = self.ax_acc_l if role == "left" else self.ax_acc_r
            self.acc_lines[role]['x'], = ax_acc.plot([], [], lw=1, label=f"x-axis", color='red', alpha=0.7)
            self.acc_lines[role]['y'], = ax_acc.plot([], [], lw=1, label=f"y-axis", color='green', alpha=0.7)
            self.acc_lines[role]['z'], = ax_acc.plot([], [], lw=1, label=f"z-axis", color='blue', alpha=0.7)
            ax_acc.legend(loc='upper right', fontsize=legend_fontsize)
            
            # 최적화: vline 객체 풀 초기화
            ax_gyro = self.ax_gyro_l if role == "left" else self.ax_gyro_r
            self._create_vline_pool(ax_gyro, self.gyro_hs_vlines_pool[role], color='red')
            self._create_vline_pool(ax_gyro, self.gyro_to_vlines_pool[role], color='blue')
        
        self.fig.canvas.mpl_connect('close_event', self._on_close)
    
        ax_reset_btn = self.fig.add_axes([0.45, 0.01, 0.1, 0.05])
        self.reset_button = Button(ax_reset_btn, 'Reset Data')
        self.reset_button.on_clicked(self._on_reset)

        self.anim = animation.FuncAnimation(
            self.fig,
            self._update,
            fargs=(analyzer,),
            interval=self.ANIMATION_INTERVAL,  # 최적화: 상수 사용
            cache_frame_data=False
        )

        print("Plotter thread started. Displaying window...")
        
        try:
            plt.show() 
        except Exception as e:
            print(f"Plot window error: {e}")
        finally:
            self._is_closed = True 

    def _on_reset(self, event):
        print("Reset button clicked.")

        # XdpcHandler의 패킷 버퍼도 함께 클리어
        if self.xdpc_handler_ref:
            self.xdpc_handler_ref.clear_packet_buffers()

        if self.analyzer_ref:
            self.analyzer_ref.reset_data()

        for role in self.sensor_roles:
            self.traj_lines[role].set_data([], [])
            self.traj_hs_scatters[role].set_offsets(np.empty((0, 2)))
            self.traj_to_scatters[role].set_offsets(np.empty((0, 2)))
            
            # stride length annotation 제거
            for ann in self.traj_stride_annotations[role]:
                ann.remove()
            self.traj_stride_annotations[role] = []
            
            self.gyro_lines[role].set_data([], [])
            
            # 최적화: vline 제거 대신 비활성화 (재사용)
            for line in self.gyro_hs_vlines_pool[role]:
                line.set_visible(False)
            for line in self.gyro_to_vlines_pool[role]:
                line.set_visible(False)
            self.gyro_hs_vlines_active[role] = 0
            self.gyro_to_vlines_active[role] = 0

            self.acc_lines[role]['x'].set_data([], [])
            self.acc_lines[role]['y'].set_data([], [])
            self.acc_lines[role]['z'].set_data([], [])

        for ax in [self.ax_gyro_l, self.ax_gyro_r, self.ax_acc_l, self.ax_acc_r]:
            ax.set_xlim(0, 10)


    def _update(self, frame, analyzer):
        
        try:
            analyzer.update_realtime_analysis()
        except Exception as e:
            print(f"Error during realtime analysis update: {e}")
            
        try:
            realtime_data = analyzer.get_realtime_data()
        except Exception as e:
            print(f"Error getting realtime plot data: {e}")
            return
            
        if not realtime_data:
            return

        max_time_in_chunk = 0
        
        for role in self.sensor_roles:
            data = realtime_data.get(role)
            if not data or data["time_array"].size == 0:
                continue

            time_array = data["time_array"]
            if time_array.size > 0 and time_array[-1] > max_time_in_chunk:
                max_time_in_chunk = time_array[-1]

            gyr_z_signal = data["gyr_z"]
            acc_xyz_signal = data["acc_xyz"]
            hs_indices = data["heel_strikes_idx"]
            to_indices = data["toe_offs_idx"]
            position = data["position"]
            position_hs_idx = data["position_hs_idx"]
            position_to_idx = data["position_to_idx"]
            
            if role == "left":
                ax_gyro = self.ax_gyro_l
                ax_acc = self.ax_acc_l
                ax_traj = self.ax_traj_l
            else:
                ax_gyro = self.ax_gyro_r
                ax_acc = self.ax_acc_r
                ax_traj = self.ax_traj_r

            # Gyro plot 업데이트
            self.gyro_lines[role].set_data(time_array, gyr_z_signal)
            
            # 최적화: vline 객체를 제거/재생성하지 않고 재사용
            if hs_indices:
                hs_times = time_array[hs_indices]
                self._update_vlines(self.gyro_hs_vlines_pool[role], 
                                   self.gyro_hs_vlines_active, role, hs_times)
            else:
                self._update_vlines(self.gyro_hs_vlines_pool[role], 
                                   self.gyro_hs_vlines_active, role, [])

            if to_indices:
                to_times = time_array[to_indices]
                self._update_vlines(self.gyro_to_vlines_pool[role], 
                                   self.gyro_to_vlines_active, role, to_times)
            else:
                self._update_vlines(self.gyro_to_vlines_pool[role], 
                                   self.gyro_to_vlines_active, role, [])
            
            # Acc plot 업데이트
            if acc_xyz_signal.shape[0] > 0 and acc_xyz_signal.shape[1] == 3:
                self.acc_lines[role]['x'].set_data(time_array, acc_xyz_signal[:, 0])
                self.acc_lines[role]['y'].set_data(time_array, acc_xyz_signal[:, 1])
                self.acc_lines[role]['z'].set_data(time_array, acc_xyz_signal[:, 2])
            
            # Trajectory plot 업데이트 (실시간)
            if position.size > 0:
                self.traj_lines[role].set_data(position[:, 0], position[:, 1])
                
                # HS 마커 업데이트
                if position_hs_idx:
                    hs_positions = position[position_hs_idx, :2]
                    self.traj_hs_scatters[role].set_offsets(hs_positions)
                else:
                    self.traj_hs_scatters[role].set_offsets(np.empty((0, 2)))
                
                # TO 마커 업데이트
                if position_to_idx:
                    to_positions = position[position_to_idx, :2]
                    self.traj_to_scatters[role].set_offsets(to_positions)
                else:
                    self.traj_to_scatters[role].set_offsets(np.empty((0, 2)))
                
                # Stride length annotation 업데이트
                # 기존 annotation 제거
                for ann in self.traj_stride_annotations[role]:
                    ann.remove()
                self.traj_stride_annotations[role] = []
                
                # 새로운 stride length 계산 및 표시
                if len(position_hs_idx) >= 2:
                    for i in range(1, len(position_hs_idx)):
                        prev_hs_idx = position_hs_idx[i-1]
                        curr_hs_idx = position_hs_idx[i]
                        
                        # 두 HS 점의 좌표
                        prev_hs_pos = position[prev_hs_idx, :2]
                        curr_hs_pos = position[curr_hs_idx, :2]
                        
                        # Stride length 계산 (XY 평면상 직선거리)
                        stride_length = np.linalg.norm(curr_hs_pos - prev_hs_pos)
                        
                        # 텍스트 위치: 끝 HS 점에서 약간 오프셋
                        # 이전 HS에서 현재 HS로의 방향 벡터에 수직인 방향으로 오프셋
                        direction_vec = curr_hs_pos - prev_hs_pos
                        if np.linalg.norm(direction_vec) > 0:
                            direction_vec = direction_vec / np.linalg.norm(direction_vec)
                            # 90도 회전 (수직 방향)
                            perpendicular_vec = np.array([-direction_vec[1], direction_vec[0]])
                            text_offset = perpendicular_vec * 0.2  # 15cm 오프셋
                        else:
                            text_offset = np.array([0.2, 0])
                        
                        text_pos = curr_hs_pos + text_offset
                        
                        # 텍스트 표시
                        text_ann = ax_traj.text(
                            text_pos[0], text_pos[1], 
                            f"{stride_length:.2f} m",
                            fontsize=10,
                            ha='left',
                            va='center',
                            color=self.colors.get(role, "gray"),
                            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=self.colors.get(role, "gray"), alpha=0.7)
                        )
                        
                        # 연결선 표시 (HS 점에서 텍스트까지)
                        line_ann, = ax_traj.plot(
                            [curr_hs_pos[0], text_pos[0]], 
                            [curr_hs_pos[1], text_pos[1]],
                            color=self.colors.get(role, "gray"),
                            linestyle=':',
                            linewidth=0.8,
                            alpha=0.5
                        )
                        
                        self.traj_stride_annotations[role].append(text_ann)
                        self.traj_stride_annotations[role].append(line_ann)
                
                # Trajectory plot auto-scale
                ax_traj.relim()
                ax_traj.autoscale_view(True, True, True)

        for ax in [self.ax_gyro_l, self.ax_gyro_r, self.ax_acc_l, self.ax_acc_r]:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
            ax.set_xlim(0, self.analyzer_ref.realtime_window_sec)

    def _on_close(self, event):
        print("Plot window closed by user.")
        self._is_closed = True

    def is_window_closed(self):
        return self._is_closed