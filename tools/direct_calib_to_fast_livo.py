#!/usr/bin/env python3
"""从标定 JSON 生成 FAST-LIVO 的外参和相机内参 YAML 配置。

用法:
    python3 direct_calib_to_fast_livo.py \
        /home/guo/direct_lidar_calib_ws/mid360_color_processed/calib.json \
        -o config/mid360_calibrated.yaml

输入 JSON 中需包含:
    {"results": {"T_lidar_camera": [x, y, z, qx, qy, qz, qw]}}

输出文件以 config/mid360.yaml 和 config/camera_mynteye.yaml 为模板，
分别更新外参 Rcl、Pcl 和相机内参。
"""

import argparse
import json
import math
from pathlib import Path
import re


def quaternion_to_rotation(qx, qy, qz, qw):
    """将 xyzw 顺序的四元数转换为 3×3 旋转矩阵。"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0:
        raise ValueError("四元数不能为零。")

    qx, qy, qz, qw = (qx / norm, qy / norm, qz / norm, qw / norm)
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]


def invert_transform(transform):
    """求 T_lidar_camera 的逆，即 T_camera_lidar。"""
    if len(transform) != 7:
        raise ValueError("T_lidar_camera 必须为 [x, y, z, qx, qy, qz, qw]。")

    x, y, z, qx, qy, qz, qw = map(float, transform)
    rotation = quaternion_to_rotation(qx, qy, qz, qw)
    rotation_inverse = [list(row) for row in zip(*rotation)]
    translation_inverse = [
        -(rotation_inverse[0][0] * x + rotation_inverse[0][1] * y + rotation_inverse[0][2] * z),
        -(rotation_inverse[1][0] * x + rotation_inverse[1][1] * y + rotation_inverse[1][2] * z),
        -(rotation_inverse[2][0] * x + rotation_inverse[2][1] * y + rotation_inverse[2][2] * z),
    ]
    return rotation_inverse, translation_inverse


def format_values(values):
    """格式化为 mid360.yaml 中使用的浮点数列表。"""
    return ", ".join(f"{value:.12g}" for value in values)


def replace_calibration(template, rotation, translation):
    """只替换 YAML 中的 Rcl 和 Pcl 数值，保留其他全部内容。"""
    flattened_rotation = [value for row in rotation for value in row]
    replacements = {
        "Rcl": f"  Rcl: [{format_values(flattened_rotation)}]",
        "Pcl": f"  Pcl: [{format_values(translation)}]",
    }

    for key, replacement in replacements.items():
        pattern = rf"(?m)^([ \t]*{key}:[ \t]*)\[[^\]]*\]"
        template, count = re.subn(pattern, replacement, template)
        if count != 1:
            raise ValueError(f"模板中应恰好包含一个 {key}，实际找到 {count} 个。")
    return template


def replace_camera_parameters(template, camera):
    """将 JSON 相机内参与前四个畸变参数写入相机 YAML 模板。"""
    try:
        fx, fy, cx, cy = map(float, camera["intrinsics"])
        d0, d1, d2, d3 = map(float, camera["distortion_coeffs"][:4])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "JSON 中 camera.intrinsics 需含 4 项，camera.distortion_coeffs 需至少含 4 项。"
        ) from error

    replacements = {
        "cam_model": "cam_model: Pinhole",
        "cam_fx": f"cam_fx: {fx:.12g}",
        "cam_fy": f"cam_fy: {fy:.12g}",
        "cam_cx": f"cam_cx: {cx:.12g}",
        "cam_cy": f"cam_cy: {cy:.12g}",
        "cam_d0": f"cam_d0: {d0:.12g}",
        "cam_d1": f"cam_d1: {d1:.12g}",
        "cam_d2": f"cam_d2: {d2:.12g}",
        "cam_d3": f"cam_d3: {d3:.12g}",
    }
    for key, replacement in replacements.items():
        pattern = rf"(?m)^[ \t]*{key}:[^\n]*"
        template, count = re.subn(pattern, replacement, template)
        if count != 1:
            raise ValueError(f"相机模板中应恰好包含一个 {key}，实际找到 {count} 个。")
    return template


def main():
    parser = argparse.ArgumentParser(description="从标定 JSON 生成 FAST-LIVO 的 YAML 标定配置")
    parser.add_argument("json_file", type=Path, help="包含 results.T_lidar_camera 的 JSON 文件")
    parser.add_argument(
        "-t",
        "--template",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "mid360.yaml",
        help="mid360.yaml 模板路径（默认：仓库中的 config/mid360.yaml）",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "mid360_calibrated.yaml",
        help="生成的 YAML 路径（默认：config/mid360_calibrated.yaml）",
    )
    parser.add_argument(
        "--camera-template",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "camera_mynteye.yaml",
        help="相机 YAML 模板路径（默认：仓库中的 config/camera_mynteye.yaml）",
    )
    parser.add_argument(
        "--camera-output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "camera_mynteye_calibrated.yaml",
        help="生成的相机 YAML 路径（默认：config/camera_mynteye_calibrated.yaml）",
    )
    args = parser.parse_args()

    with args.json_file.open(encoding="utf-8") as file:
        data = json.load(file)

    try:
        transform = data["results"]["T_lidar_camera"]
    except (KeyError, TypeError) as error:
        raise ValueError("JSON 中未找到 results.T_lidar_camera。") from error

    rotation, translation = invert_transform(transform)
    output_content = replace_calibration(
        args.template.read_text(encoding="utf-8"), rotation, translation
    )
    args.output.write_text(output_content, encoding="utf-8")
    print(f"已生成: {args.output}")

    camera_content = replace_camera_parameters(
        args.camera_template.read_text(encoding="utf-8"), data.get("camera", {})
    )
    args.camera_output.write_text(camera_content, encoding="utf-8")
    print(f"已生成: {args.camera_output}")


if __name__ == "__main__":
    main()

