#include "bladerf_ros2/bladerf_source_node.hpp"

#include <libbladeRF.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <stdexcept>

namespace bladerf_ros2
{

  // SC16 Q11: 12-bit values stored in 16-bit containers, full-scale = 2^11 = 2048.
  static constexpr float SC16_SCALE = 1.0f / 2048.0f;

  // ---------------------------------------------------------------------------
  // Constructor / Destructor
  // ---------------------------------------------------------------------------

  BladeRFSourceNode::BladeRFSourceNode(const rclcpp::NodeOptions &options)
      : Node("bladerf_source", options)
  {
    device_str_         = declare_parameter<std::string>("device", "");
    fpga_path_          = declare_parameter<std::string>("fpga", "");
    frequency_          = declare_parameter<double>("frequency", 915e6);
    sample_rate_        = declare_parameter<double>("sample_rate", 10e6);
    bandwidth_          = declare_parameter<double>("bandwidth", 8e6);
    gain_               = declare_parameter<int>("gain", 30);
    split_count_        = declare_parameter<int>("split_count", 1);
    samples_per_switch_ = declare_parameter<int>("samples_per_switch", 4096);
    num_buffers_        = declare_parameter<int>("num_buffers", 512);
    samples_per_buffer_ = declare_parameter<int>("samples_per_buffer", 4096);
    num_transfers_      = declare_parameter<int>("num_transfers", 32);
    stream_timeout_ms_  = declare_parameter<int>("stream_timeout_ms", 3000);

    // incremented with each buffer published, used to track dropped samples
    qos_index = 0;

    if (split_count_ < 1)        split_count_ = 1;
    if (samples_per_switch_ < 1) samples_per_switch_ = 1;

    // MIMO raw buffer layout from bladerf_sync_rx with BLADERF_RX_X2:
    //   [RX0_I0, RX0_Q0, RX1_I0, RX1_Q0,  RX0_I1, RX0_Q1, RX1_I1, RX1_Q1, ...]
    //
    // bladerf_sync_rx is told to receive samples_per_buffer_ samples *per channel*.
    // The library writes 2 channels x 2 int16 x samples_per_buffer_ int16 values total.
    raw_buf_.resize(4 * samples_per_buffer_);

    // Each accumulator holds one switch period for one channel:
    //   2 floats (I+Q) x samples_per_switch_ complex samples
    acc_ref_.resize(2 * samples_per_switch_);
    acc_switched_.resize(2 * samples_per_switch_);

    pub_ = create_publisher<bladerf_ros2::msg::IQSamples>("iq_samples", 10);

    if (!open_device())
    {
      throw std::runtime_error("Failed to open bladeRF device");
    }

    running_ = true;
    stream_thread_ = std::thread(&BladeRFSourceNode::stream_loop, this);
  }

  BladeRFSourceNode::~BladeRFSourceNode()
  {
    running_ = false;
    if (stream_thread_.joinable())
    {
      stream_thread_.join();
    }
    close_device();
  }

  // ---------------------------------------------------------------------------
  // Device lifecycle
  // ---------------------------------------------------------------------------

  bool BladeRFSourceNode::open_device()
  {
    int status;

    const char *devstr = device_str_.empty() ? nullptr : device_str_.c_str();
    status = bladerf_open(&dev_, devstr);
    if (status != 0)
    {
      RCLCPP_ERROR(get_logger(), "bladerf_open failed: %s", bladerf_strerror(status));
      return false;
    }

    if (!fpga_path_.empty())
    {
      status = bladerf_load_fpga(dev_, fpga_path_.c_str());
      if (status != 0)
      {
        RCLCPP_WARN(get_logger(), "bladerf_load_fpga failed: %s", bladerf_strerror(status));
      }
    }

    // Configure both RX channels identically (same frequency, rate, bandwidth, gain).
    for (int ch_idx = 0; ch_idx < 2; ++ch_idx)
    {
      bladerf_channel ch = BLADERF_CHANNEL_RX(ch_idx);

      status = bladerf_set_frequency(dev_, ch, static_cast<bladerf_frequency>(frequency_));
      if (status != 0)
      {
        RCLCPP_ERROR(get_logger(), "bladerf_set_frequency (RX%d) failed: %s",
                     ch_idx, bladerf_strerror(status));
        bladerf_close(dev_); dev_ = nullptr; return false;
      }

      bladerf_sample_rate actual_rate = 0;
      status = bladerf_set_sample_rate(
          dev_, ch, static_cast<bladerf_sample_rate>(sample_rate_), &actual_rate);
      if (status != 0)
      {
        RCLCPP_ERROR(get_logger(), "bladerf_set_sample_rate (RX%d) failed: %s",
                     ch_idx, bladerf_strerror(status));
        bladerf_close(dev_); dev_ = nullptr; return false;
      }
      RCLCPP_INFO(get_logger(), "RX%d actual sample rate: %u sps", ch_idx, actual_rate);

      bladerf_bandwidth actual_bw = 0;
      status = bladerf_set_bandwidth(
          dev_, ch, static_cast<bladerf_bandwidth>(bandwidth_), &actual_bw);
      if (status != 0)
      {
        RCLCPP_ERROR(get_logger(), "bladerf_set_bandwidth (RX%d) failed: %s",
                     ch_idx, bladerf_strerror(status));
        bladerf_close(dev_); dev_ = nullptr; return false;
      }
      RCLCPP_INFO(get_logger(), "RX%d actual bandwidth: %u Hz", ch_idx, actual_bw);

      status = bladerf_set_gain(dev_, ch, static_cast<bladerf_gain>(gain_));
      if (status != 0)
      {
        RCLCPP_WARN(get_logger(), "bladerf_set_gain (RX%d) failed: %s",
                    ch_idx, bladerf_strerror(status));
      }
    }

    // Configure MIMO synchronous streaming.
    // BLADERF_RX_X2: both RX channels active, interleaved as channel pairs.
    status = bladerf_sync_config(
        dev_,
        BLADERF_RX_X2,
        BLADERF_FORMAT_SC16_Q11,
        static_cast<unsigned int>(num_buffers_),
        static_cast<unsigned int>(samples_per_buffer_),
        static_cast<unsigned int>(num_transfers_),
        static_cast<unsigned int>(stream_timeout_ms_));
    if (status != 0)
    {
      RCLCPP_ERROR(get_logger(), "bladerf_sync_config failed: %s", bladerf_strerror(status));
      bladerf_close(dev_); dev_ = nullptr; return false;
    }

    // Enable both RX modules.
    for (int ch_idx = 0; ch_idx < 2; ++ch_idx)
    {
      status = bladerf_enable_module(dev_, BLADERF_CHANNEL_RX(ch_idx), true);
      if (status != 0)
      {
        RCLCPP_ERROR(get_logger(), "bladerf_enable_module (RX%d) failed: %s",
                     ch_idx, bladerf_strerror(status));
        bladerf_close(dev_); dev_ = nullptr; return false;
      }
    }

    RCLCPP_INFO(
        get_logger(),
        "bladeRF opened (MIMO): freq=%.3f MHz  rate=%.3f Msps  bw=%.3f MHz  gain=%d dB"
        "  split_count=%d  samples_per_switch=%d"
        "  RX0=reference  RX1=switched",
        frequency_ / 1e6, sample_rate_ / 1e6, bandwidth_ / 1e6, gain_,
        split_count_, samples_per_switch_);

    return true;
  }

  void BladeRFSourceNode::close_device()
  {
    if (dev_)
    {
      bladerf_enable_module(dev_, BLADERF_CHANNEL_RX(0), false);
      bladerf_enable_module(dev_, BLADERF_CHANNEL_RX(1), false);
      bladerf_close(dev_);
      dev_ = nullptr;
    }
  }

  // ---------------------------------------------------------------------------
  // Streaming thread
  // ---------------------------------------------------------------------------
  //
  // MIMO wire format (BLADERF_RX_X2, BLADERF_FORMAT_SC16_Q11):
  //
  //   raw_buf_:
  //   [ RX0_I0  RX0_Q0  RX1_I0  RX1_Q0       <- frame 0
  //     RX0_I1  RX0_Q1  RX1_I1  RX1_Q1       <- frame 1
  //     ...                               ]
  //
  //   Each "frame" = 4 int16_t values = one complex sample from each channel.
  //
  //   bladerf_sync_rx receives samples_per_buffer_ frames, so the total
  //   int16_t values in raw_buf_ = 4 * samples_per_buffer_.
  //
  // Design:
  //   Walk frame-by-frame, converting SC16->float32 on the fly and appending
  //   to acc_ref_ (RX0) and acc_switched_ (RX1) in lock-step via the shared
  //   cursor acc_pos_.  When acc_pos_ reaches 2*samples_per_switch_ (floats)
  //   both accumulators hold a complete switch period and we publish one
  //   IQSamples message containing both.
  //
  // ---------------------------------------------------------------------------

  void BladeRFSourceNode::stream_loop()
  {
    const int floats_per_period = 2 * samples_per_switch_;

    current_antenna_idx_ = 0;
    acc_pos_             = 0;

    while (running_)
    {
      // ----------------------------------------------------------------------
      // 1.  Receive one MIMO buffer from hardware.
      //     Count passed to bladerf_sync_rx = samples_per_buffer_ (per channel).
      //     Library fills 4 * samples_per_buffer_ int16_t values into raw_buf_.
      // ----------------------------------------------------------------------
      int status = bladerf_sync_rx(
          dev_,
          static_cast<void *>(raw_buf_.data()),
          static_cast<unsigned int>(samples_per_buffer_),
          nullptr,
          static_cast<unsigned int>(stream_timeout_ms_));

      if (status != 0)
      {
        RCLCPP_WARN(get_logger(), "bladerf_sync_rx error: %s", bladerf_strerror(status));
        continue;
      }

      // ----------------------------------------------------------------------
      // 2.  Walk frame-by-frame through the interleaved MIMO buffer.
      //
      //     Frame layout (indices into raw_buf_ at frame*4):
      //       +0  RX0 I  (reference)
      //       +1  RX0 Q  (reference)
      //       +2  RX1 I  (switched antenna)
      //       +3  RX1 Q  (switched antenna)
      // ----------------------------------------------------------------------
      for (int frame = 0; frame < samples_per_buffer_; ++frame)
      {
        const int16_t * f = raw_buf_.data() + frame * 4;

        // Convert this frame to float32 for both channels.
        acc_ref_[acc_pos_]          = static_cast<float>(f[0]) * SC16_SCALE; // RX0 I
        acc_ref_[acc_pos_ + 1]      = static_cast<float>(f[1]) * SC16_SCALE; // RX0 Q
        acc_switched_[acc_pos_]     = static_cast<float>(f[2]) * SC16_SCALE; // RX1 I
        acc_switched_[acc_pos_ + 1] = static_cast<float>(f[3]) * SC16_SCALE; // RX1 Q

        acc_pos_ += 2;

        // --------------------------------------------------------------------
        // 3.  Switch period complete?
        // --------------------------------------------------------------------
        if (acc_pos_ == floats_per_period)
        {
          // ------------------------------------------------------------------
          // 4.  Publish one message with both the switched and reference data
          //     for this completed switch period.
          //
          //     switched_samples  = RX1 (switched antenna, antenna_index)
          //     reference_samples = RX0 (reference, always streaming)
          //     antenna_index     = current switch position (0 … split_count-1)
          // ------------------------------------------------------------------
          auto msg = std::make_unique<bladerf_ros2::msg::IQSamples>();

          msg->header.stamp      = now();
          msg->header.frame_id   = "bladerf";
          msg->antenna_index     = static_cast<uint32_t>(current_antenna_idx_);
          msg->sample_rate       = static_cast<uint32_t>(sample_rate_);
          msg->center_frequency  = static_cast<uint32_t>(frequency_);
          msg->indx = this->qos_index;

          msg->switched_samples.assign(acc_switched_.begin(), acc_switched_.end());
          msg->reference_samples.assign(acc_ref_.begin(), acc_ref_.end());

          pub_->publish(std::move(msg));
          this->qos_index +=1;

          // ------------------------------------------------------------------
          // 5.  Advance switch state machine and reset write cursor.
          // ------------------------------------------------------------------
          acc_pos_ = 0;
          current_antenna_idx_ = (current_antenna_idx_ + 1) % split_count_;
        }
      }
    }
  }

} // namespace bladerf_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(bladerf_ros2::BladeRFSourceNode)