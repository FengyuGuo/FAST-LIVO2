#!/usr/bin/env python3
"""使用 Open3D 对 PCD 做体素降采样后转换为 PLY，并保留已有的 RGB 颜色。

用法：
    python3 tools/pcd_to_ply.py input.pcd
    python3 tools/pcd_to_ply.py input.pcd --voxel-size 0.1 -o output.ply
"""

import argparse
from pathlib import Path

try:
    import open3d as o3d
except ImportError as error:
    raise SystemExit(
        "未安装 Open3D。请先执行：python3 -m pip install open3d"
    ) from error


def main():
    parser = argparse.ArgumentParser(description="体素降采样 PCD 后转换为保留颜色的 PLY")
    parser.add_argument("input_pcd", type=Path, help="输入 PCD 文件")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="输出 PLY 文件路径（默认与输入文件同名，仅扩展名改为 .ply）",
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="输出 ASCII 格式 PLY（默认输出二进制 PLY）",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="体素边长，单位米（默认：0.1）",
    )
    args = parser.parse_args()

    if args.input_pcd.suffix.lower() != ".pcd":
        parser.error("输入文件必须为 .pcd 格式。")
    if not args.input_pcd.is_file():
        parser.error(f"找不到输入文件：{args.input_pcd}")
    if args.voxel_size <= 0:
        parser.error("--voxel-size 必须大于 0。")

    output_path = args.output or args.input_pcd.with_suffix(".ply")
    if output_path.suffix.lower() != ".ply":
        parser.error("输出文件必须使用 .ply 扩展名。")

    point_cloud = o3d.io.read_point_cloud(str(args.input_pcd))
    if point_cloud.is_empty():
        raise RuntimeError(f"未能从 {args.input_pcd} 读取到点。")
    original_count = len(point_cloud.points)
    point_cloud = point_cloud.voxel_down_sample(args.voxel_size)

    if not o3d.io.write_point_cloud(
        str(output_path),
        point_cloud,
        write_ascii=args.ascii,
        compressed=False,
        print_progress=True,
    ):
        raise RuntimeError(f"写入 PLY 失败：{output_path}")

    color_message = "保留 RGB 颜色" if point_cloud.has_colors() else "输入点云不含 RGB 颜色"
    print(
        f"转换完成：{output_path}（体素 {args.voxel_size:g} m，"
        f"{original_count} → {len(point_cloud.points)} 点，{color_message}）"
    )


if __name__ == "__main__":
    main()
