#!/usr/bin/env python3
"""
generate_rosbag_metadata.py — 为缺少 metadata.yaml 的 rosbag 目录生成元数据文件

用途：
  某些 rosbag 录制后只有 .db3 文件而没有 metadata.yaml，
  导致 `ros2 bag play` 无法使用。此脚本从 .db3 文件中提取
  topic 信息并生成标准的 metadata.yaml。

用法：
  python3 scripts/generate_rosbag_metadata.py /path/to/rosbag/directory
  # 或指定 db3 文件：
  python3 scripts/generate_rosbag_metadata.py /path/to/rosbag.db3

示例：
  python3 scripts/generate_rosbag_metadata.py \\
    /home/rm/rq/projects/radar/lidar-rosbag/competition/全明星赛第一局bag/全明星赛第一局/
"""

import os
import sys
import sqlite3

try:
    import yaml
except ImportError:
    print("需要 pyyaml: pip install pyyaml", file=sys.stderr)
    sys.exit(1)


def generate_metadata(db3_path: str, output_dir: str):
    """从 db3 文件生成 metadata.yaml"""
    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()

    # 获取 topic 信息
    cursor.execute(
        'SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics'
    )
    topics = cursor.fetchall()

    topic_info = []
    for tid, name, ttype, ser_fmt, qos in topics:
        cursor.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?', (tid,))
        count = cursor.fetchone()[0]
        topic_info.append({
            'id': tid, 'name': name, 'type': ttype,
            'serialization_format': ser_fmt, 'count': count,
            'qos': qos
        })

    # 获取时间范围
    cursor.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages')
    start_time, end_time = cursor.fetchone()

    # 总消息数
    cursor.execute('SELECT COUNT(*) FROM messages')
    total_count = cursor.fetchone()[0]

    conn.close()

    # 构建 metadata
    db3_filename = os.path.basename(db3_path)
    topics_with_meta = []
    for t in topic_info:
        entry = {
            'topic_metadata': {
                'name': t['name'],
                'type': t['type'],
                'serialization_format': t['serialization_format'],
            },
            'message_count': t['count'],
        }
        if t['qos']:
            entry['topic_metadata']['offered_qos_profiles'] = t['qos']
        topics_with_meta.append(entry)

    metadata = {
        'rosbag2_bagfile_information': {
            'version': 8,
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [db3_filename],
            'duration': {'nanoseconds': end_time - start_time},
            'starting_time': {'nanoseconds_since_epoch': start_time},
            'message_count': total_count,
            'topics_with_message_count': topics_with_meta,
            'compression_format': '',
            'compression_mode': '',
            'files': [
                {
                    'path': db3_filename,
                    'starting_time': {'nanoseconds_since_epoch': start_time},
                    'duration': {'nanoseconds': end_time - start_time},
                    'message_count': total_count,
                }
            ],
            'custom_data': {},
        }
    }

    output_path = os.path.join(output_dir, 'metadata.yaml')
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(metadata, f, default_flow_style=False, allow_unicode=True)

    print(f'✅ metadata.yaml 已生成: {output_path}')
    print(f'   时长: {(end_time - start_time) / 1e9:.1f} 秒')
    print(f'   消息总数: {total_count}')
    print(f'   Topics:')
    for t in topic_info:
        print(f'     {t["name"]:30s}  {t["type"]:50s}  count={t["count"]}')


def main():
    if len(sys.argv) < 2:
        print(f'用法: {sys.argv[0]} <rosbag_directory_or_db3_file>')
        sys.exit(1)

    path = sys.argv[1]

    if os.path.isdir(path):
        # 在目录中查找 .db3 文件
        db3_files = [f for f in os.listdir(path) if f.endswith('.db3')]
        if not db3_files:
            print(f'❌ 在 {path} 中未找到 .db3 文件', file=sys.stderr)
            sys.exit(1)
        if len(db3_files) > 1:
            print(f'⚠ 找到多个 .db3 文件，使用第一个: {db3_files[0]}')
        db3_path = os.path.join(path, db3_files[0])
        output_dir = path
    elif os.path.isfile(path) and path.endswith('.db3'):
        db3_path = path
        output_dir = os.path.dirname(path)
    else:
        print(f'❌ 无效路径: {path}', file=sys.stderr)
        sys.exit(1)

    # 检查是否已存在 metadata.yaml
    existing_meta = os.path.join(output_dir, 'metadata.yaml')
    if os.path.exists(existing_meta):
        answer = input(f'⚠ {existing_meta} 已存在，是否覆盖？[y/N] ')
        if answer.lower() != 'y':
            print('取消操作')
            sys.exit(0)

    generate_metadata(db3_path, output_dir)


if __name__ == '__main__':
    main()
