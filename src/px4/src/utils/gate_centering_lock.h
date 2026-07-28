#pragma once

#include <array>
#include <vector>

namespace px4
{

// ==================================================================
// GateCenteringLock
//
// Kelas murni (tanpa rclcpp), sama seperti VisionLock/GroundLock —
// tidak tahu apa pun soal ROS atau px4_msgs. Sumbernya titik body-
// frame dari Livox MID360s (/livox/points, PointCloud2), sudah
// diekstrak jadi array x/y/z oleh ControlModule sebelum masuk sini.
//
// Diporting dari centeringGateLivoxSimple() di
// centering_ws/src/control/utils/control_.cpp (versi MAVROS/SITL) —
// matematikanya (ROI filter + binning lateral + deteksi dua puncak
// tiang gerbang) dipertahankan, tapi di sini murni fungsi
// stateless-per-update: tidak melakukan spin/loop/publish sendiri.
// Pemanggil (MissionManager, lewat ControlModule::LivoxCallback)
// yang memutuskan mau dipakai untuk apa (velocity command langsung
// seperti versi asli, atau locked-target seperti VisionLock) —
// lihat catatan integrasi di gate_centering_lock.cpp.
//
// Konvensi sumbu (frame sensor Livox, sama seperti kode asli):
//   forward = +X, lateral = +Y (kanan positif), height = Z.
// ==================================================================

class GateCenteringLock
{
public:
    struct Config
    {
        // ROI (region of interest) di frame sensor Livox.
        float roi_forward_min_m{1.0f};
        float roi_forward_max_m{10.0f};
        float roi_lateral_m{3.0f};       // ±m dari tengah
        float roi_height_min_m{-1.0f};
        float roi_height_max_m{2.0f};

        // Gerbang yang dicari.
        float gate_width_m{1.5f};
        float centering_tolerance_m{0.2f};
        float target_forward_distance_m{1.75f};

        // Binning lateral untuk deteksi dua tiang (puncak histogram).
        int num_bins{60};
        int min_cluster_points{5};

        // Kalau hanya satu tiang yang kelihatan (gerbang di tepi ROI),
        // tetap coba centering ke sisi itu alih-alih diam saja.
        bool allow_one_side_centering{true};
    };

    // Hasil satu update() — snapshot deteksi frame ini, tidak ada
    // debounce/lock internal (beda dari VisionLock: gerbang Livox
    // butuh update tiap frame karena gerbangnya statis, bukan target
    // yang perlu di-freeze saat hilang sesaat).
    struct Result
    {
        bool valid{false};          // cukup titik untuk ambil keputusan
        bool one_side_only{false};  // hanya satu tiang terdeteksi

        // Body-frame Livox: +Y kanan, error positif = gerbang di
        // kanan sensor -> perlu geser kanan (vy_body positif).
        float lateral_error_m{0.0f};
        float forward_distance_m{0.0f};
        float detected_width_m{0.0f};
        bool  width_valid{false};

        bool centered{false};       // |lateral_error_m| < tolerance frame ini
    };

    explicit GateCenteringLock(const Config & config);

    // points: titik body-frame Livox (x=forward, y=lateral, z=height),
    // sudah difilter non-finite oleh caller ATAU dibiarkan (update()
    // sendiri sudah skip NaN/Inf).
    Result update(const std::vector<std::array<float, 3>> & points) const;

private:
    Config config_;
};

}  // namespace px4
