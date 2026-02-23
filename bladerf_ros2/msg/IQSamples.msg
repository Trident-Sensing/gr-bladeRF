# One block of IQ samples from a the switched and reference antenna element.
#
# samples is interleaved complex float32: [i0, q0, i1, q1, ...]
# matching GNU Radio's native gr_complex (complex64) memory layout.
# Reconstruct: np.frombuffer(msg.samples, dtype=np.float32).view(np.complex64)
# The reference_samples field contains samples from a fixed reference antenna, used for phase calibration.
# The reference samples should be the same length as the switched samples, and corresponds to the same time period.
# Block length == samples_per_switch.  All samples in one message are from
# the same antenna (antenna_index) during one switch period.
std_msgs/Header header
float32[]  switched_samples        # interleaved IQ: [i0, q0, i1, q1, ...]
float32[]  reference_samples       # interleaved IQ: [i0, q0, i1, q1, ...]
uint32     antenna_index           # which switched antenna this block is from (0-indexed)
uint32     sample_rate
uint32     center_frequency