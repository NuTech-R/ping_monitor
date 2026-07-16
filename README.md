# ping_monitor
ROS2トピックの往復時間(RTT)で通信リンクの遅延・ロスを監視するパッケージ。

計測側の `topic_ping_node` が `link_ping/request` にseq入りHeaderをpublishし、
対向の `topic_echo_node` がそのまま `link_ping/response` に返す。
測定は計測側の時計だけで完結するため、対向との時刻同期は不要。

# Usage
```bash
# 応答側
ros2 run ping_monitor topic_echo_node
# 計測側
ros2 run ping_monitor topic_ping_node
```

# Topics
| Topic                | Type               | Description                                    |
| -------------------- | ------------------ | ---------------------------------------------- |
| `/ping_latency`      | `std_msgs/Float32` | RTT in ms (`-1.0` on timeout).                 |
| `/ping_packet_loss`  | `std_msgs/Float32` | 直近の要求ごとに `0.0`(応答あり) / `100.0`(タイムアウト)。 |
| `/link_ping/request` | `std_msgs/Header`  | 測定用要求 (best_effort)。frame_idにseq。       |
| `/link_ping/response`| `std_msgs/Header`  | echoが返す応答 (best_effort)。                 |

送信間隔(0.5s)とタイムアウト(1.0s)は `topic_ping_node.py` 内の定数。
