#pragma once

#include <atomic>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <bladerf_ros2/msg/iq_samples.hpp>

// Forward-declare the bladeRF C struct so we don't need the header here.
struct bladerf;

namespace bladerf_ros2 {

/**
 * BladeRFSourceNode
 *
 * ROS2 composable node that opens a bladeRF SDR device, receives SC16 Q11
 * samples via bladerf_sync_rx, and publishes one IQSamples message per
 * completed antenna-switch period.
 *
 * Parameters
 * ----------
 * device            (string,  "")      bladeRF device string (empty = first found)
 * fpga              (string,  "")      Path to FPGA bitstream (.rbf); skipped if empty
 * frequency         (double,  915e6)   Center frequency [Hz]
 * sample_rate       (double,  10e6)    Sample rate [sps]
 * bandwidth         (double,  8e6)     RX filter bandwidth [Hz]
 * gain              (int,     30)      Overall RX gain [dB]
 * split_count       (int,     1)       Number of antenna switch positions
 * samples_per_switch(int,     4096)    Samples collected per switch position
 * num_buffers       (int,     512)     USB streaming buffers
 * samples_per_buffer(int,     4096)    Samples per USB buffer (multiple of 1024)
 * num_transfers     (int,     32)      Active USB transfers
 * stream_timeout_ms (int,     3000)    bladerf_sync_rx timeout [ms]
 *
 * Published topic
 * ---------------
 * ~/iq_samples  (bladerf_ros2/IQSamples)
 *   One message per completed switch period.  samples.size() == 2*samples_per_switch
 *   (interleaved float32 I/Q).  antenna_index cycles 0 … split_count-1.
 */
class BladeRFSourceNode : public rclcpp::Node {
public:
  explicit BladeRFSourceNode(const rclcpp::NodeOptions & options);
  ~BladeRFSourceNode() override;

private:
  // ---------- device lifecycle ----------
  bool open_device();
  void close_device();

  // ---------- streaming ----------
  void stream_loop();

  // ---------- bladeRF handle ----------
  struct bladerf * dev_{nullptr};

  // ---------- parameters ----------
  std::string device_str_;
  std::string fpga_path_;
  double      frequency_;
  double      sample_rate_;
  double      bandwidth_;
  int         gain_;
  int         split_count_;
  int         samples_per_switch_;
  int         num_buffers_;
  int         samples_per_buffer_;
  int         num_transfers_;
  int         stream_timeout_ms_;

  // ---------- buffers ----------
  std::vector<int16_t> raw_buf_;      // SC16 Q11 from hardware: [I0 Q0 I1 Q1 ...]
  std::vector<float>   acc_ref_;      // float32 IQ accumulating for RX0 (reference)
  std::vector<float>   acc_switched_; // float32 IQ accumulating for RX1 (switched)

  // ---------- antenna-switch state machine ----------
  int current_antenna_idx_{0};
  int acc_pos_{0};   // next write position (in floats) inside accumulator_
  int qos_index;

  // ---------- streaming thread ----------
  std::thread        stream_thread_;
  std::atomic<bool>  running_{false};

  // ---------- publisher ----------
  rclcpp::Publisher<bladerf_ros2::msg::IQSamples>::SharedPtr pub_;
};

}  // namespace bladerf_ros2
