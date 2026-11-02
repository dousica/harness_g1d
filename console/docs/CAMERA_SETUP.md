# G1-D 相机配置（本机实测，2026-11-01）

## 结论：当前默认配置

图像服务跑在机器人 `192.168.123.164`（用户 `unitree`，sudo 密码同登录密码），
systemd 单元 `teleimager.service`，配置在：

```
/home/unitree/unitree_eai_environment/service/teleimager/cam_config_server.yaml
```

注意：**不在** `~/.config/teleimager/`（服务以 root 启动，早期文档写的路径在这台机器上不存在）。

| 相机 | image_shape | 每目分辨率 | fps | ZMQ 端口 |
|---|---|---|---|---|
| head_camera（双目并排） | `[1080, 3840]` | 1920×1080 | 10 | 55555 |
| left_wrist_camera | `[1080, 1920]` | 1920×1080 | 30 | 55556 |
| right_wrist_camera | `[1080, 1920]` | 1920×1080 | 30 | 55557 |

`image_shape` 是 `[高, 宽]`；双目相机总宽 = 2 × 单目宽。

## USB 带宽是唯一约束

三颗相机共用一个 USB 控制器，实测只有下列组合稳定：

| 组合 | head | 手腕 | 结果 |
|---|---|---|---|
| A（当前） | 1080p/目 @10 | 1080p @30 | ✅ 全最高分辨率 |
| B | 1080p/目 @15 | 720p @30 | ✅ 头部更流畅 |
| — | 1080p/目 @15 | 1080p @30 | ❌ `libusb_set_interface_alt_setting failed` |
| — | 1080p/目 @30 | 任意 | ❌ 服务崩溃重启 |

关键：限制来自 **head 的帧率占带宽**，不是分辨率本身。head 15→10 fps 正好腾出足够带宽让两只手腕上 1080p。

## 相机真实能力（`teleimager-server --cf`，需先停服务再探测）

- head（serial `01.00.00`）：双目模式最高 `1080x3840`、`720x2560` 等，30/50/60 fps 均列出
- 手腕（`JR0001`/`JR0002`）：最高 `1080x1920`，仅支持 30/60 fps（**没有 15 fps**，写 15 会报 `No matching uvc mode found`）

## 修改与回退

```bash
ssh unitree@192.168.123.164
F=/home/unitree/unitree_eai_environment/service/teleimager/cam_config_server.yaml
# 改 image_shape / fps 后
sudo systemctl restart teleimager
```

原始配置（640×480/目）备份：`cam_config_server.yaml.bak.1793540613`

服务反复崩溃时先 `sudo systemctl reset-failed teleimager` 再 start。

## 验证

```bash
cd ~/桌面/gpt_harness
PYTHONPATH=$PWD unitree_gpt6/g1d_draw/.venv/bin/python \
  unitree_gpt6/g1d_draw/examples/capture_images.py \
  --stream head=tcp://192.168.123.164:55555 \
  --stream left_wrist=tcp://192.168.123.164:55556 \
  --stream right_wrist=tcp://192.168.123.164:55557 \
  --output captures/view --timeout 60
```

## 其他环境备注

- `unitree_sdk2py` 的 pip 包缺 `utils/lib/crc_amd64.so`；从仓库
  `unitree_gpt6/g1d_gomoku/src/unitree_sdk2py/utils/lib/` 复制到 site-packages 即可。
- 系统缺 `python3-venv`，用 `uv venv --python /usr/bin/python3.10` 建环境。
- `start.bash` 只启动 ROS 相机**客户端**，采集服务始终在 192.168.123.164 上。
