"""Qt GUI for recording, saving, and playing four-axis robot arm poses."""

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = (
    ('Base', 'base_shoulder'),
    ('Shoulder', 'shoulder_arm1'),
    ('Forearm', 'arm1_arm2'),
    ('Upper arm', 'arn2_end_arm'),
)
MIN_ANGLE = 60
MAX_ANGLE = 120
CENTER_ANGLE = 90


class JointStatePublisher(Node):
    """Publish servo-degree GUI values as URDF joint radians."""

    def __init__(self):
        super().__init__('ros_arm_sequence_gui')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def publish_angles(self, servo_angles):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [joint_name for _, joint_name in JOINTS]
        message.position = [
            math.radians(angle - CENTER_ANGLE) for angle in servo_angles
        ]
        self.publisher.publish(message)


class SequenceWindow(QMainWindow):
    """Four sliders plus an editable pose sequence and JSON persistence."""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.sliders = []
        self.angle_labels = []
        self.sequence = []
        self.playing = False
        self.paused = False
        self.play_index = 0
        self.step_started_at = 0.0
        self.step_start_angles = [CENTER_ANGLE] * len(JOINTS)
        self.current_file = None

        self.setWindowTitle('ROS 2 Robot Arm Sequence GUI')
        self.resize(920, 650)
        self._build_ui()

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._publish_current_pose)
        self.publish_timer.start(100)

        self.play_timer = QTimer(self)
        self.play_timer.timeout.connect(self._advance_playback)
        self.play_timer.start(20)

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)

        control_group = QGroupBox('관절 제어 (실제 서보 각도)')
        control_layout = QGridLayout(control_group)
        for row, (label, _) in enumerate(JOINTS):
            name = QLabel(label)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(MIN_ANGLE, MAX_ANGLE)
            slider.setValue(CENTER_ANGLE)
            slider.setTickInterval(5)
            slider.setTickPosition(QSlider.TicksBelow)
            value = QLabel('90°')
            value.setMinimumWidth(48)
            slider.valueChanged.connect(
                lambda angle, index=row: self._angle_changed(index, angle))
            self.sliders.append(slider)
            self.angle_labels.append(value)
            control_layout.addWidget(name, row, 0)
            control_layout.addWidget(slider, row, 1)
            control_layout.addWidget(value, row, 2)

        center_button = QPushButton('모두 90° 중심')
        center_button.clicked.connect(self._center)
        control_layout.addWidget(center_button, len(JOINTS), 1)
        layout.addWidget(control_group)

        pose_row = QHBoxLayout()
        pose_row.addWidget(QLabel('단계 이름'))
        self.pose_name = QLineEdit()
        self.pose_name.setPlaceholderText('예: 물체 잡기 직전')
        pose_row.addWidget(self.pose_name, 1)
        pose_row.addWidget(QLabel('이동 시간(초)'))
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.1, 30.0)
        self.duration.setSingleStep(0.1)
        self.duration.setValue(1.0)
        pose_row.addWidget(self.duration)
        add_button = QPushButton('현재 자세 추가')
        add_button.clicked.connect(self._add_pose)
        pose_row.addWidget(add_button)
        update_button = QPushButton('선택 단계 덮어쓰기')
        update_button.clicked.connect(self._update_pose)
        pose_row.addWidget(update_button)
        layout.addLayout(pose_row)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ['단계 이름', 'Base', 'Shoulder', 'Forearm', 'Upper', '시간(s)'])
        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked)
        self.table.cellChanged.connect(self._table_edited)
        self.table.cellDoubleClicked.connect(self._load_row_pose)
        layout.addWidget(self.table, 1)

        edit_row = QHBoxLayout()
        up_button = QPushButton('위로')
        up_button.clicked.connect(lambda: self._move_row(-1))
        down_button = QPushButton('아래로')
        down_button.clicked.connect(lambda: self._move_row(1))
        delete_button = QPushButton('선택 삭제')
        delete_button.clicked.connect(self._delete_pose)
        clear_button = QPushButton('전체 삭제')
        clear_button.clicked.connect(self._clear_sequence)
        for button in (up_button, down_button, delete_button, clear_button):
            edit_row.addWidget(button)
        edit_row.addStretch()
        layout.addLayout(edit_row)

        playback = QHBoxLayout()
        play_button = QPushButton('▶ 재생')
        play_button.clicked.connect(self._play)
        pause_button = QPushButton('⏸ 일시정지/계속')
        pause_button.clicked.connect(self._toggle_pause)
        stop_button = QPushButton('■ 정지')
        stop_button.clicked.connect(self._stop)
        self.repeat = QCheckBox('반복 재생')
        save_button = QPushButton('JSON 저장')
        save_button.clicked.connect(self._save)
        load_button = QPushButton('JSON 불러오기')
        load_button.clicked.connect(self._load)
        for widget in (
                play_button, pause_button, stop_button, self.repeat,
                save_button, load_button):
            playback.addWidget(widget)
        layout.addLayout(playback)

        self.status = QLabel('준비됨 — 자세를 만든 뒤 “현재 자세 추가”를 누르세요.')
        layout.addWidget(self.status)
        self.setCentralWidget(root)

    def _angles(self):
        return [slider.value() for slider in self.sliders]

    def _set_angles(self, angles):
        for slider, angle in zip(self.sliders, angles):
            slider.setValue(max(MIN_ANGLE, min(MAX_ANGLE, round(angle))))

    def _angle_changed(self, index, angle):
        self.angle_labels[index].setText(f'{angle}°')
        if not self.playing:
            self.status.setText(
                '수동 조작 중 — 한 관절씩 천천히 움직이세요.')

    def _center(self):
        self._stop()
        self._set_angles([CENTER_ANGLE] * len(JOINTS))
        self.status.setText('모든 관절을 90° 중심으로 이동했습니다.')

    def _publish_current_pose(self):
        self.ros_node.publish_angles(self._angles())
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)

    def _new_pose(self):
        name = self.pose_name.text().strip()
        if not name:
            name = f'pose_{len(self.sequence) + 1}'
        return {
            'name': name,
            'angles': self._angles(),
            'duration': round(self.duration.value(), 2),
        }

    def _add_pose(self):
        self.sequence.append(self._new_pose())
        self.pose_name.clear()
        self._refresh_table(select_row=len(self.sequence) - 1)
        self.status.setText(f'{len(self.sequence)}번째 자세를 추가했습니다.')

    def _selected_row(self):
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def _update_pose(self):
        row = self._selected_row()
        if row < 0:
            QMessageBox.information(self, '단계 선택', '덮어쓸 단계를 먼저 선택하세요.')
            return
        pose = self._new_pose()
        if not self.pose_name.text().strip():
            pose['name'] = self.sequence[row]['name']
        self.sequence[row] = pose
        self._refresh_table(select_row=row)
        self.status.setText(f'{row + 1}번째 단계를 현재 자세로 덮어썼습니다.')

    def _refresh_table(self, select_row=-1):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.sequence))
        for row, pose in enumerate(self.sequence):
            values = [
                pose['name'],
                *[str(value) for value in pose['angles']],
                f"{pose['duration']:.2f}",
            ]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))
        self.table.blockSignals(False)
        if 0 <= select_row < len(self.sequence):
            self.table.selectRow(select_row)

    def _table_edited(self, row, column):
        if not (0 <= row < len(self.sequence)):
            return
        item = self.table.item(row, column)
        try:
            if column == 0:
                self.sequence[row]['name'] = item.text().strip() or f'pose_{row + 1}'
            elif 1 <= column <= 4:
                angle = max(MIN_ANGLE, min(MAX_ANGLE, int(item.text())))
                self.sequence[row]['angles'][column - 1] = angle
            else:
                duration = max(0.1, min(30.0, float(item.text())))
                self.sequence[row]['duration'] = round(duration, 2)
        except ValueError:
            QMessageBox.warning(self, '잘못된 값', '각도는 정수, 시간은 숫자로 입력하세요.')
        self._refresh_table(select_row=row)

    def _load_row_pose(self, row, _column):
        self._stop()
        pose = self.sequence[row]
        self._set_angles(pose['angles'])
        self.pose_name.setText(pose['name'])
        self.duration.setValue(pose['duration'])
        self.status.setText(f'{row + 1}번째 자세를 슬라이더에 불러왔습니다.')

    def _move_row(self, direction):
        row = self._selected_row()
        destination = row + direction
        if row < 0 or not (0 <= destination < len(self.sequence)):
            return
        self.sequence[row], self.sequence[destination] = (
            self.sequence[destination], self.sequence[row])
        self._refresh_table(select_row=destination)

    def _delete_pose(self):
        row = self._selected_row()
        if row >= 0:
            del self.sequence[row]
            self._refresh_table(select_row=min(row, len(self.sequence) - 1))

    def _clear_sequence(self):
        if not self.sequence:
            return
        answer = QMessageBox.question(
            self, '전체 삭제', '저장하지 않은 모든 단계를 삭제할까요?')
        if answer == QMessageBox.Yes:
            self._stop()
            self.sequence.clear()
            self._refresh_table()

    def _play(self):
        if not self.sequence:
            QMessageBox.information(self, '시퀀스 없음', '자세를 한 개 이상 추가하세요.')
            return
        self.playing = True
        self.paused = False
        self.play_index = 0
        self._start_step()

    def _start_step(self):
        self.step_start_angles = self._angles()
        self.step_started_at = time.monotonic()
        self.table.selectRow(self.play_index)
        pose = self.sequence[self.play_index]
        self.status.setText(
            f'재생 {self.play_index + 1}/{len(self.sequence)}: {pose["name"]}')

    def _advance_playback(self):
        if not self.playing or self.paused:
            return
        pose = self.sequence[self.play_index]
        elapsed = time.monotonic() - self.step_started_at
        progress = min(1.0, elapsed / pose['duration'])
        interpolated = [
            start + (target - start) * progress
            for start, target in zip(self.step_start_angles, pose['angles'])
        ]
        self._set_angles(interpolated)
        if progress < 1.0:
            return
        self.play_index += 1
        if self.play_index >= len(self.sequence):
            if self.repeat.isChecked():
                self.play_index = 0
            else:
                self.playing = False
                self.status.setText('시퀀스 재생 완료.')
                return
        self._start_step()

    def _toggle_pause(self):
        if not self.playing:
            return
        self.paused = not self.paused
        if self.paused:
            self.status.setText('일시정지 — 현재 자세를 유지합니다.')
        else:
            self.step_start_angles = self._angles()
            self.step_started_at = time.monotonic()
            self.status.setText('재생을 계속합니다.')

    def _stop(self):
        self.playing = False
        self.paused = False
        self.status.setText('재생 정지 — 현재 자세를 유지합니다.')

    def _sequence_document(self):
        return {
            'format_version': 1,
            'name': (
                self.current_file.stem if self.current_file else 'robot_arm_sequence'
            ),
            'angle_unit': 'servo_degree',
            'center_angle': CENTER_ANGLE,
            'limits': [MIN_ANGLE, MAX_ANGLE],
            'joints': [joint_name for _, joint_name in JOINTS],
            'steps': self.sequence,
        }

    def _save(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, '시퀀스 JSON 저장', str(Path.home() / 'robot_arm_sequence.json'),
            'JSON files (*.json)')
        if not filename:
            return
        if not filename.lower().endswith('.json'):
            filename += '.json'
        try:
            Path(filename).write_text(
                json.dumps(self._sequence_document(), ensure_ascii=False, indent=2)
                + '\n',
                encoding='utf-8',
            )
        except OSError as error:
            QMessageBox.critical(self, '저장 실패', str(error))
            return
        self.current_file = Path(filename)
        self.status.setText(f'저장 완료: {filename}')

    def _load(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, '시퀀스 JSON 불러오기', str(Path.home()),
            'JSON files (*.json)')
        if not filename:
            return
        try:
            document = json.loads(Path(filename).read_text(encoding='utf-8'))
            steps = document['steps']
            validated = []
            for index, pose in enumerate(steps):
                angles = [int(value) for value in pose['angles']]
                if len(angles) != len(JOINTS):
                    raise ValueError(f'{index + 1}번째 단계의 각도 개수가 다릅니다.')
                if any(not MIN_ANGLE <= value <= MAX_ANGLE for value in angles):
                    raise ValueError(
                        f'{index + 1}번째 단계가 {MIN_ANGLE}~{MAX_ANGLE}°를 벗어났습니다.')
                duration = float(pose['duration'])
                if not 0.1 <= duration <= 30.0:
                    raise ValueError(f'{index + 1}번째 이동 시간이 범위를 벗어났습니다.')
                validated.append({
                    'name': str(pose.get('name', f'pose_{index + 1}')),
                    'angles': angles,
                    'duration': round(duration, 2),
                })
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            QMessageBox.critical(self, '불러오기 실패', str(error))
            return
        self._stop()
        self.sequence = validated
        self.current_file = Path(filename)
        self._refresh_table(select_row=0 if validated else -1)
        self.status.setText(f'{len(validated)}개 단계를 불러왔습니다: {filename}')

    def closeEvent(self, event):
        self._stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    app = QApplication(sys.argv)
    window = SequenceWindow(node)
    window.show()
    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
