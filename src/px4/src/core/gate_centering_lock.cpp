#include "utils/gate_centering_lock.h"

#include <algorithm>
#include <cmath>

namespace px4
{

GateCenteringLock::GateCenteringLock(const Config & config)
: config_(config)
{}

GateCenteringLock::Result GateCenteringLock::update(
    const std::vector<std::array<float, 3>> & points) const
{
    Result result;

    const float x_min = config_.roi_forward_min_m;
    const float x_max = config_.roi_forward_max_m;
    const float y_min = -config_.roi_lateral_m;
    const float y_max = config_.roi_lateral_m;
    const float z_min = config_.roi_height_min_m;
    const float z_max = config_.roi_height_max_m;

    const int num_bins = std::max(1, config_.num_bins);
    const float bin_size = (y_max - y_min) / static_cast<float>(num_bins);
    const int min_peak_separation = std::max(
        1, static_cast<int>((config_.gate_width_m * 0.5f) / bin_size));
    // Log lapangan menunjukkan gate asli konsisten 1.6--1.8 m untuk target
    // 1.5 m, sedangkan pasangan clutter palsu muncul di 0.8--1.1 m atau
    // >3 m. Toleransi lama ±1 m terlalu longgar dan membuat clutter dianggap
    // dua tiang gate. Batasi sekitar 25% lebar nominal (min 25 cm, max 45 cm).
    const float gate_width_tolerance = std::clamp(
        config_.gate_width_m * 0.25f, 0.25f, 0.45f);
    const float gate_width_min = std::max(
        0.5f, config_.gate_width_m - gate_width_tolerance);
    const float gate_width_max =
        config_.gate_width_m + gate_width_tolerance;

    std::vector<int> bins(num_bins, 0);
    std::vector<float> forward_sum_bins(num_bins, 0.0f);
    int total_roi_points = 0;

    for (const auto & p : points) {
        const float forward = p[0];
        const float lateral = p[1];
        const float height = p[2];
        if (!std::isfinite(forward) || !std::isfinite(lateral) || !std::isfinite(height)) {
            continue;
        }
        if (forward < x_min || forward > x_max ||
            lateral < y_min || lateral > y_max ||
            height < z_min || height > z_max) {
            continue;
        }

        const int bin_idx = static_cast<int>((lateral - y_min) / bin_size);
        if (bin_idx >= 0 && bin_idx < num_bins) {
            bins[bin_idx]++;
            forward_sum_bins[bin_idx] += forward;
            total_roi_points++;
        }
    }

    int first_peak_idx = -1;
    int first_peak_count = 0;
    for (int i = 0; i < num_bins; ++i) {
        if (bins[i] > first_peak_count) {
            first_peak_count = bins[i];
            first_peak_idx = i;
        }
    }

    int second_peak_idx = -1;
    int second_peak_count = 0;
    for (int i = 0; i < num_bins; ++i) {
        if (std::abs(i - first_peak_idx) < min_peak_separation) {
            continue;
        }
        if (bins[i] > second_peak_count) {
            second_peak_count = bins[i];
            second_peak_idx = i;
        }
    }

    // Hanya satu tiang terdeteksi (gerbang di tepi ROI) — kalau
    // diizinkan, tetap kasih koreksi menuju sisi seharusnya.
    if (config_.allow_one_side_centering &&
        total_roi_points >= config_.min_cluster_points &&
        first_peak_count >= config_.min_cluster_points &&
        second_peak_count < config_.min_cluster_points) {
        const float peak_lateral = y_min + (first_peak_idx + 0.5f) * bin_size;
        const float desired_side_lateral =
            (peak_lateral >= 0.0f ? 1.0f : -1.0f) * config_.gate_width_m * 0.5f;

        result.valid = true;
        result.one_side_only = true;
        result.lateral_error_m = peak_lateral - desired_side_lateral;
        result.forward_distance_m =
            forward_sum_bins[first_peak_idx] / std::max(1, bins[first_peak_idx]);
        result.width_valid = false;
        result.centered = std::abs(result.lateral_error_m) < config_.centering_tolerance_m;
        return result;
    }

    // Tidak cukup titik di salah satu/kedua puncak -> tidak ada
    // keputusan valid frame ini.
    if (total_roi_points < config_.min_cluster_points * 2 ||
        first_peak_count < config_.min_cluster_points ||
        second_peak_count < config_.min_cluster_points) {
        return result;
    }

    const int left_idx = std::min(first_peak_idx, second_peak_idx);
    const int right_idx = std::max(first_peak_idx, second_peak_idx);
    const float left_y = y_min + (left_idx + 0.5f) * bin_size;
    const float right_y = y_min + (right_idx + 0.5f) * bin_size;
    const float detected_width = right_y - left_y;
    const float left_forward = forward_sum_bins[left_idx] / std::max(1, bins[left_idx]);
    const float right_forward = forward_sum_bins[right_idx] / std::max(1, bins[right_idx]);

    result.valid = true;
    result.one_side_only = false;
    result.lateral_error_m = (left_y + right_y) * 0.5f;
    result.forward_distance_m = 0.5f * (left_forward + right_forward);
    result.detected_width_m = detected_width;
    result.width_valid = detected_width >= gate_width_min && detected_width <= gate_width_max;
    // Vektor dari tiang kiri ke kanan adalah (dx, dy). Normal gate yang
    // mengarah ke depan sensor adalah (dy, -dx): untuk gate lurus di depan,
    // dy > 0 dan dx = 0 sehingga error yaw = 0. Ini hanya MELAPORKAN
    // geometri yang sama; deteksi/centering lateral di atas tidak diubah.
    result.heading_error_rad = std::atan2(
        -(right_forward - left_forward), detected_width);
    result.heading_valid = result.width_valid;
    result.centered = result.width_valid &&
        std::abs(result.lateral_error_m) < config_.centering_tolerance_m;

    return result;
}

}  // namespace px4
