#!/usr/bin/python3.6
# encoding=utf-8
"""
python tools/plot_log.py -f /apollo/data/log/planning.INFO -t 11:50:34
"""

import argparse
from collections import defaultdict
import os
import re
import shutil
import sys
import time
import math
import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Cursor
from matplotlib.gridspec import GridSpec
from matplotlib import ticker


def get_string_between(string, st, ed=''):
    """get string between keywords"""
    if string.find(st) < 0:
        return ''
    sub_string = string[string.find(st) + len(st):]
    if len(ed) == 0 or sub_string.find(ed) < 0:
        return sub_string.strip()
    return sub_string[:sub_string.find(ed)]


def get_planning_seq_num(line):
    """get planning seq num from line"""
    return get_string_between(line, 'start frame sequence id = [', ']')


def get_time(line):
    """get time from line"""
    return get_string_between(line, ' ', ' ')


def get_lines_between(lines, st, ed=''):
    """get valid log with keywords"""
    valid_lines = []
    found_start = False
    for line in lines:
        if st in line:
            found_start = True
        if len(ed) != 0 and ed in line:
            break
        if found_start:
            valid_lines.append(line)
    return valid_lines


def get_points_from_line(line, data):
    pat = re.compile(r'[(](.*?)[)]', re.S)
    str_list = re.findall(pat, line)
    for string in str_list:
        num = string.split(",")
        data.append([float(num[0]), float(num[1])])


marker_map = {'roi_boundary': '-',
              'vehicle_start_box': '-',
              'end_position': 'o',
              'vehicle_end_box': '-',
              'rs_point': '*',
              'warm_path': '-',
              }


def transform(origin_pt, origin_heading, points):
    for point in points:
        tmp_x = point[0]
        print("before", origin_pt, origin_heading, point)
        point[0] = point[0] * \
            math.cos(origin_heading) - point[0] * math.sin(origin_heading)
        point[1] = tmp_x * math.sin(origin_heading) + \
            point[1] * math.cos(origin_heading)
        point[0] += origin_pt[0]
        point[1] += origin_pt[1]
        print("after", origin_pt, origin_heading, point)


def conver_point_to_xy(points):
    x = []
    y = []
    for pt in points:
        x.append(pt[0])
        y.append(pt[1])
    return x, y


def plot_open_space(lines, ax, origin_pt, origin_heading):
    elem_map = {}
    for key in marker_map.keys():
        elem_map[key] = []
    for line in lines:
        for key in marker_map.keys():
            name = "print_" + key + ":"
            if name in line:
                get_points_from_line(line, elem_map[key])
                transform(origin_pt, origin_heading, elem_map[key])

    for key in elem_map.keys():
        if elem_map[key]:
            x, y = conver_point_to_xy(elem_map[key])
            ax.plot(x, y, marker_map[key], label=key)


def find_origin_point(lines):
    x = 0.0
    y = 0.0
    data = []
    for line in lines:
        if "origin_point: " in line:
            get_points_from_line(line, data)
            return data[0]


def find_origin_heading(lines):
    for line in lines:
        if "origin_heading:" in line:
            return float(get_string_between(line, "origin_heading:", ","))


def plot_frame(fig, ax, lines, line_st_num, line_ed_num):
    """plot ref frame"""
    print('plot line start num: ' + str(line_st_num + 1))
    print('plot line end num: ' + str(line_ed_num + 1))
    valid_lines = []
    for i in range(line_st_num, line_ed_num):
        valid_lines.append(lines[i])

    for value in ax.values():
        value.legend()
        value.grid(True)
    origin_pt = find_origin_point(valid_lines)
    origin_heading = find_origin_heading(valid_lines)
    print("origin point", origin_pt)
    print("origin_heading", origin_heading)
    plot_open_space(valid_lines, ax['open_space'], origin_pt, origin_heading)
    ax['open_space'].set_aspect(1)

    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

    plt.draw()

    return


def search_next(lines, line_search_num, keyword):
    for i in range(line_search_num, len(lines)):
        if keyword in lines[i]:
            return i
    return 0


def search_last(lines, line_search_num, keyword):
    for i in range(line_search_num, 0, -1):
        if keyword in lines[i]:
            return i
    return 0


def search_keyword_line(lines, keyword):
    """search line with time"""
    for i in range(len(lines)):
        if keyword in lines[i]:
            return i
    return 0


class MouseEventManager(object):
    x, y = 0.0, 0.0
    xoffset, yoffset = -20, 20
    text_template = 'x: %0.2f\ny: %0.2f'
    annotation = False

    def on_click(self, event):
        # if mouse button is not right, return
        # 1: left, 2: middle, 3: right
        if event.button is not 3:
            return
        self.x, self.y = event.xdata, event.ydata
        if self.x is not None:
            print('mouse click x: %.2f, y: %.2f' % (event.xdata, event.ydata))
            if self.annotation:
                self.annotation.set_visible(False)
            label_text = self.text_template % (self.x, self.y)
            self.annotation = event.inaxes.annotate(label_text,
                                                    xy=(self.x, self.y), xytext=(
                                                        self.xoffset, self.yoffset),
                                                    textcoords='offset points', ha='right', va='bottom',
                                                    bbox=dict(
                                                        boxstyle='round,pad=0.5', fc='lightcyan', alpha=0.5),
                                                    arrowprops=dict(
                                                        arrowstyle='->', connectionstyle='arc3,rad=0')
                                                    )
            self.annotation.set_visible(True)
            self.annotation.figure.canvas.draw()


class Index(object):
    """button callback function"""

    def __init__(self, fig, ax, line_st_num, line_ed_num, lines):
        self.ax = ax
        self.fig = fig
        self.line_st_num = line_st_num
        self.line_ed_num = line_ed_num
        self.lines = lines
        self.reset_mouse_event()

    def reset_mouse_event(self):
        self.mouse_manager = MouseEventManager()
        fig.canvas.mpl_connect('button_release_event',
                               self.mouse_manager.on_click)

    def next(self, step):
        """next button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id = search_next(
                self.lines, self.line_ed_num + 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach last line, may reach last frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines,
                   self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def next1(self, event):
        self.next(1)

    def next10(self, event):
        self.next(10)

    def prev(self, step):
        """prev button callback function"""
        for i in range(step):
            line_st_num, line_ed_num, seq_id = search_last(
                self.lines, self.line_st_num - 1)
            # check whether found frame log is complete
            if line_st_num < 0 or line_ed_num < 0:
                print('[ERROR] search reach first line, may reach first frame')
                return
            self.line_st_num = line_st_num
            self.line_ed_num = line_ed_num
        plot_frame(fig, self.ax, self.lines,
                   self.line_st_num, self.line_ed_num)
        self.reset_mouse_event()

    def prev1(self, event):
        self.prev(1)

    def prev10(self, event):
        self.prev(10)

    def exit(self, event):
        sys.exit(0)


if __name__ == '__main__':
    global g_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', action='store', dest='log_file_path',
                        required=True, help='log file path')
    g_argv = parser.parse_args()
    print('Please wait for loading data...')

    # load log file
    print(g_argv)
    file_path = g_argv.log_file_path
    # file_path = parse_pb_log(file_path)
    input = open(file_path, 'r')
    lines = input.readlines()
    line_st_num = search_keyword_line(lines, " origin_point:")
    line_ed_num = search_keyword_line(lines, "print_warm_path:") + 1

    # check whether time is exist
    if line_st_num == 0 or line_ed_num == 0 :
        print('no keyword, quit!', line_st_num, line_ed_num)
        sys.exit(0)

    fig = plt.figure(figsize=[9, 15])
    gs = GridSpec(1, 1, figure=fig)
    ax = {}
    ax['open_space'] = fig.add_subplot(gs[0, 0])

    plot_frame(fig, ax, lines, line_st_num, line_ed_num)

    callback = Index(fig, ax, line_st_num, line_ed_num, lines)
    prev1frame = plt.axes([0.2, 0.01, 0.1, 0.05])
    prev10frame = plt.axes([0.3, 0.01, 0.1, 0.05])
    next1frame = plt.axes([0.4, 0.01, 0.1, 0.05])
    next10frame = plt.axes([0.5, 0.01, 0.1, 0.05])
    exitframe = plt.axes([0.6, 0.01, 0.1, 0.05])
    bprev1 = Button(prev1frame, '-1')
    bprev1.on_clicked(callback.prev1)
    bprev10 = Button(prev10frame, '-10')
    bprev10.on_clicked(callback.prev10)
    bnext1 = Button(next1frame, '+1')
    bnext1.on_clicked(callback.next1)
    bnext10 = Button(next10frame, '+10')
    bnext10.on_clicked(callback.next10)
    bexit = Button(exitframe, 'exit')
    bexit.on_clicked(callback.exit)
    plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    plt.gca().yaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    plt.show()
    plt.ion()
