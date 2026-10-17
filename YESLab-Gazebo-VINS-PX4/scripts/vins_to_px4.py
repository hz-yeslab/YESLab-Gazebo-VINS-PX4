#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vins_to_px4.py —— 把 VINS-Fusion 的位姿送入 PX4 外部视觉接口（含显式坐标转换）

数据流：
    VINS-Fusion -> /vins_estimator/odometry -> 本节点 -> /mavros/vision_pose/pose -> PX4 EKF2

========================= 坐标系转换（本任务的核心之一） =========================

ROS / VINS 惯例：世界系 ENU（x 东, y 北, z 上），机体系 FLU（前-左-上）
PX4 惯例：        世界系 NED（x 北, y 东, z 下），机体系 FRD（前-右-下）

位置（世界系，ENU -> NED）：
    x_ned =  y_enu
    y_ned =  x_enu
    z_ned = -z_enu

姿态（机体 FRD 相对 NED，由 ENU/FLU 的四元数换算）：
    R_ned_frd = C * R_enu_flu * C ,   其中 C = [[0,1,0],[1,0,0],[0,0,-1]]
    等价直观写法：roll_frd = -roll_flu, pitch_frd = pitch_flu, yaw_frd = -yaw_flu

MAVROS 的 /mavros/vision_pose/pose 期望的就是【NED 世界系 + FRD 机体姿态】。

=============================== 自检（无需 ROS） ===============================
    python3 scripts/vins_to_px4.py --selftest

用法（ROS 环境）：
    rosrun vins_bridge vins_to_px4.py
    rosrun vins_bridge vins_to_px4.py _frame_mode:=passthrough
"""

from __future__ import print_function

import math
import sys

# ---------------------------------------------------------------- 纯数学部分

# ENU -> NED 换基矩阵 C = [[0,1,0],[1,0,0],[0,0,-1]]
_C = ((0.0, 1.0, 0.0),
      (1.0, 0.0, 0.0),
      (0.0, 0.0, -1.0))


def enu_to_ned_position(p):
    """世界系位置 ENU -> NED。"""
    x, y, z = p
    return (y, x, -z)


def _mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _mat_transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _quat_to_matrix(q):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
        (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)),
    )


def _matrix_to_quat(m):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。"""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def enu_flu_quat_to_ned_frd(q):
    """姿态四元数：机体相对世界（ENU/FLU） -> 机体相对世界（NED/FRD）。"""
    r = _quat_to_matrix(q)
    r_ned_frd = _mat_mul(_mat_mul(_C, r), _C)   # C 对称且 C^-1 = C
    return _matrix_to_quat(r_ned_frd)


def enu_to_ned_euler_equivalent(q):
    """直观等价写法：roll' = -roll, pitch' = pitch, yaw' = -yaw（仅用于校验）。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return (-roll, pitch, -yaw)


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _rpy_from_quat(q):
    m = _quat_to_matrix(q)
    roll = math.atan2(m[2][1], m[2][2])
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    yaw = math.atan2(m[1][0], m[0][0])
    return (roll, pitch, yaw)


# ---------------------------------------------------------------- 自检

def _approx(a, b, tol=1e-9):
    return abs(a - b) <= tol


def _quat_close(a, b, tol=1e-6):
    same = all(_approx(x, y, tol) for x, y in zip(a, b))
    neg = all(_approx(x, -y, tol) for x, y in zip(a, b))
    return same or neg      # q 与 -q 表示同一姿态


def selftest():
    ok = True

    # 1) 位置：ENU -> NED
    cases = [((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
             ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
             ((0.0, 0.0, 2.0), (0.0, 0.0, -2.0))]
    for src, exp in cases:
        got = enu_to_ned_position(src)
        good = all(_approx(g, e) for g, e in zip(got, exp))
        ok = ok and good
        print("  [%s] position %s -> %s (期望 %s)" % ("OK" if good else "FAIL", src, got, exp))

    # 2) 姿态：90 度 yaw 的换基
    q_enu = _quat_from_rpy(0.0, 0.0, math.pi / 2.0)
    q_ned = enu_flu_quat_to_ned_frd(q_enu)
    r, p, y = _rpy_from_quat(q_ned)
    good = _approx(r, 0.0, 1e-6) and _approx(p, 0.0, 1e-6) and _approx(y, -math.pi / 2.0, 1e-6)
    ok = ok and good
    print("  [%s] yaw 90deg ENU -> rpy NED = (%.6f, %.6f, %.6f)，期望 (0, 0, -1.570796)"
          % ("OK" if good else "FAIL", r, p, y))

    # 3) 矩阵法与直观欧拉写法一致
    for (rr, pp, yy) in [(0.3, -0.2, 1.1), (-0.5, 0.4, -2.0), (0.0, 0.0, 0.0), (0.7, 0.9, 0.2)]:
        q = _quat_from_rpy(rr, pp, yy)
        qm = enu_flu_quat_to_ned_frd(q)
        want = enu_to_ned_euler_equivalent(q)
        got = _rpy_from_quat(qm)
        good = all(_approx(g, w, 1e-6) for g, w in zip(got, want))
        ok = ok and good
        print("  [%s] 等价写法 rpy=%s -> 矩阵 (%.6f, %.6f, %.6f) vs 直观 (%.6f, %.6f, %.6f)"
              % ("OK" if good else "FAIL", (rr, pp, yy), got[0], got[1], got[2], want[0], want[1], want[2]))

    # 4) 往返：ENU 姿态 -> NED 姿态 -> ENU 姿态
    for (rr, pp, yy) in [(0.2, 0.3, 0.4), (-0.6, 0.1, 2.5)]:
        q = _quat_from_rpy(rr, pp, yy)
        back = enu_flu_quat_to_ned_frd(enu_flu_quat_to_ned_frd(q))
        good = _quat_close(q, back, 1e-6)
        ok = ok and good
        print("  [%s] 往返一致 rpy=%s" % ("OK" if good else "FAIL", (rr, pp, yy)))

    print("SELFTEST: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------- ROS 节点

class VinsToPx4(object):
    """把 VINS 的 ENU/FLU 位姿转换为 NED/FRD 并发布到 MAVROS vision_pose。"""

    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped

        self._rospy = rospy
        self.input_topic = rospy.get_param("~input_topic", "/vins_estimator/odometry")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/vision_pose/pose")
        self.frame_id = rospy.get_param("~output_frame_id", "map")
        self.frame_mode = rospy.get_param("~frame_mode", "enu_to_ned")   # 或 passthrough
        self.print_every = int(rospy.get_param("~print_every", 50))
        self.axis_x, self.sign_x = int(rospy.get_param("~axis_x", 0)), float(rospy.get_param("~sign_x", 1.0))
        self.axis_y, self.sign_y = int(rospy.get_param("~axis_y", 1)), float(rospy.get_param("~sign_y", 1.0))
        self.axis_z, self.sign_z = int(rospy.get_param("~axis_z", 2)), float(rospy.get_param("~sign_z", 1.0))

        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=10)
        self.count = 0

        rospy.loginfo("[vins_to_px4] %s -> %s | frame_mode=%s | output frame_id=%s",
                      self.input_topic, self.output_topic, self.frame_mode, self.frame_id)

    def _convert(self, pos, quat):
        x, y, z = pos
        qx, qy, qz, qw = quat
        if self.frame_mode == "enu_to_ned":
            p = enu_to_ned_position((x, y, z))
            q = enu_flu_quat_to_ned_frd((qx, qy, qz, qw))
        else:
            p = (x, y, z)
            q = (qx, qy, qz, qw)

        # 逐轴微调（默认单位映射，不改动；实测方向不对时才用）
        vals = p
        ax = (self.axis_x, self.axis_y, self.axis_z)
        sg = (self.sign_x, self.sign_y, self.sign_z)
        p = tuple(sg[i] * vals[ax[i]] for i in range(3))
        return p, q

    def cb(self, odom):
        rospy = self._rospy
        from geometry_msgs.msg import PoseStamped

        pos = (odom.pose.pose.position.x,
               odom.pose.pose.position.y,
               odom.pose.pose.position.z)
        quat = (odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w)
        if any((v != v) or math.isinf(v) for v in pos) or any((v != v) or math.isinf(v) for v in quat):
            rospy.logwarn_throttle(5.0, "[vins_to_px4] 收到非有限位姿，已丢弃该帧")
            return

        p, q = self._convert(pos, quat)

        out = PoseStamped()
        out.header.stamp = odom.header.stamp if odom.header.stamp != rospy.Time(0) else rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
        self.pub.publish(out)

        self.count += 1
        if self.print_every > 0 and self.count % self.print_every == 0:
            rospy.loginfo("[vins_to_px4] #%d ENU=(%.3f, %.3f, %.3f) -> NED=(%.3f, %.3f, %.3f)",
                          self.count, pos[0], pos[1], pos[2], p[0], p[1], p[2])


def main():
    if "--selftest" in sys.argv:
        sys.exit(selftest())

    import rospy
    rospy.init_node("vins_to_px4")
    VinsToPx4()
    rospy.spin()


if __name__ == "__main__":
    main()