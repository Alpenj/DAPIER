#!/usr/bin/env python3
"""scene.xml을 GLFW 창에 띄우는 최소 뷰어 (교재 4~5장).

교재 코드에 마우스 조작(회전/이동/줌)과 R(리셋)/Space(일시정지)만 더했다.
화면이 없는 환경에서는 render_poses.py 를 쓰면 된다.
"""

from __future__ import annotations

import os
import sys

import mujoco
import glfw

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, "scene.xml")

if not glfw.init():
    sys.exit("GLFW를 초기화할 수 없습니다. (DISPLAY 확인)")

window = glfw.create_window(960, 720, "jdcobot100 - MuJoCo", None, None)
if not window:
    glfw.terminate()
    sys.exit("GLFW 창을 생성할 수 없습니다.")

glfw.make_context_current(window)
glfw.swap_interval(1)

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)

cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=2000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

mujoco.mjv_defaultCamera(cam)
mujoco.mjv_defaultOption(opt)

# 팔 전체가 15 cm 정도라 기본 거리 0.7 m는 너무 멀다.
cam.lookat[:] = [0.0, 0.0, 0.10]
cam.distance = 0.55
cam.elevation = -12
cam.azimuth = 135

state = {"lastx": 0.0, "lasty": 0.0, "left": False, "right": False, "paused": False}


def on_mouse_button(win, button, act, mods):
    state["left"] = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    state["right"] = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
    state["lastx"], state["lasty"] = glfw.get_cursor_pos(win)


def on_mouse_move(win, xpos, ypos):
    dx, dy = xpos - state["lastx"], ypos - state["lasty"]
    state["lastx"], state["lasty"] = xpos, ypos
    if not (state["left"] or state["right"]):
        return
    width, height = glfw.get_window_size(win)
    action = (mujoco.mjtMouse.mjMOUSE_ROTATE_V if state["left"]
              else mujoco.mjtMouse.mjMOUSE_MOVE_V)
    mujoco.mjv_moveCamera(model, action, dx / height, dy / height, scene, cam)


def on_scroll(win, xoff, yoff):
    mujoco.mjv_moveCamera(model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoff,
                          scene, cam)


def on_key(win, key, scancode, act, mods):
    if act != glfw.PRESS:
        return
    if key == glfw.KEY_ESCAPE:
        glfw.set_window_should_close(win, True)
    elif key == glfw.KEY_R:
        mujoco.mj_resetData(model, data)
    elif key == glfw.KEY_SPACE:
        state["paused"] = not state["paused"]


glfw.set_mouse_button_callback(window, on_mouse_button)
glfw.set_cursor_pos_callback(window, on_mouse_move)
glfw.set_scroll_callback(window, on_scroll)
glfw.set_key_callback(window, on_key)

while not glfw.window_should_close(window):
    if not state["paused"]:
        step_start = data.time
        while data.time - step_start < 1.0 / 60.0:
            mujoco.mj_step(model, data)

    width, height = glfw.get_framebuffer_size(window)
    viewport = mujoco.MjrRect(0, 0, width, height)
    mujoco.mjv_updateScene(model, data, opt, None, cam,
                           mujoco.mjtCatBit.mjCAT_ALL, scene)
    mujoco.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
