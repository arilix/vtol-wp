#pragma once

#include "utils/vehicle_state.h"
#include "utils/vision_lock.h"

namespace px4
{

// ==================================================================
// GroundLock
//
// Sama seperti VisionLock, tapi sumbernya bukan pose ArUco (solvePnP,
// sudah metrik) melainkan titik tengah box YOLO dalam piksel. Ukuran
// fisik box tidak diketahui pasti (beda dari marker ArUco yang
// ukurannya presis diisi ke solvePnP), jadi offset piksel dikonversi
// ke meter lewat proyeksi ground-plane: asumsikan target rata di
// tanah, jarak tegak lurus kamera-ke-tanah = altitude AGL (dari
// lidar), lensa pinhole ideal (fx/fy hasil kalibrasi kamera).
//
//   x_m = (cx_px - cx0_px) / fx_px * altitude_m   (kanan gambar)
//   y_m = (cy_px - cy0_px) / fy_px * altitude_m   (bawah gambar)
//
// Hasil x_m/y_m punya konvensi sama seperti tvec ArUco (frame optical
// OpenCV: x=kanan, y=bawah), jadi setelah dikonversi tinggal reuse
// VisionLock apa adanya untuk rotasi mount-yaw + yaw drone -> NED,
// termasuk debounce dan clamp magnitude-nya — supaya perilaku lock
// (freeze saat target hilang, jump-reject, dst.) identik dengan jalur
// ArUco tanpa duplikasi logic.
// ==================================================================

class GroundLock
{
public:
    struct Config
    {
        // Focal length kamera (piksel). Kalau kamera yang dipakai YOLO
        // sama persis dengan kamera ArUco, isi dengan fx/fy dari
        // src/fiducial_detector/config/params.yaml (camera_matrix).
        double camera_fx_px{640.0};
        double camera_fy_px{640.0};

        // Diteruskan apa adanya ke VisionLock internal — rotasi mount
        // yaw + drone yaw, clamp magnitude, dan debounce sample identik
        // dengan jalur ArUco.
        VisionLock::Config vision_cfg;
    };

    explicit GroundLock(const Config & config);

    // Dipanggil tiap sample /general_box/target_center datang.
    // cx_px/cy_px: titik tengah box (piksel, origin kiri-atas gambar).
    // frame_width_px/frame_height_px: dimensi frame saat ini — dipakai
    // sebagai principal point (cx0=width/2, cy0=height/2) selama belum
    // ada kalibrasi cx0/cy0 terpisah, sama seperti default camera_matrix
    // fiducial_detector.
    // altitude_m: altitude AGL sekarang (dari lidar). Sample diabaikan
    // kalau altitude tidak valid (<=0 atau non-finite) — altitude 0/
    // negatif akan menghasilkan offset nol/salah arah, bukan cuma
    // kurang akurat.
    void update(
        double cx_px, double cy_px,
        double frame_width_px, double frame_height_px,
        double altitude_m,
        const PositionNED & drone_pos, double drone_yaw,
        double now_s);

    bool isEngaged(double now_s) const { return vision_lock_.isEngaged(now_s); }
    PositionNED lockedTarget() const { return vision_lock_.lockedTarget(); }
    bool lastSampleClamped() const { return vision_lock_.lastSampleClamped(); }

private:
    Config config_;
    VisionLock vision_lock_;
};

}  // namespace px4
