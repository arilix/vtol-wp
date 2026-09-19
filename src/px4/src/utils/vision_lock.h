#pragma once

#include "utils/vehicle_state.h"

namespace px4
{

// ==================================================================
// VisionLock
//
// Kelas murni (tanpa rclcpp), sama seperti WaypointHandler — supaya
// gampang di-unit-test dan tidak tahu apa pun soal ROS.
//
// Tugasnya: ubah offset marker ArUco (dari kamera nadir, satuan meter,
// frame kamera) jadi target NED yang dikoreksi, untuk mengunci posisi
// hold (HOVER / waypoint-reached-dwell) ke marker fisik di tanah alih-
// alih mempercayai estimasi NED yang bisa drift/jitter.
//
// Strategi kunci: target = posisi_drone_sekarang + offset, DIHITUNG
// ULANG setiap sample marker baru datang (closed-loop, bukan one-shot).
// Karena posisi_drone_sekarang sendiri sudah dikompensasi drift/reset
// EKF di MissionManager, dan offset kamera adalah pengukuran independen
// ke marker fisik yang sama, penjumlahan ini otomatis menghapus drift
// NED yang terakumulasi di setiap sample segar. "Freeze saat marker
// hilang" otomatis didapat gratis: locked_target_ HANYA berubah di
// dalam update(), jadi tidak ada sample baru = tidak ada perubahan.
// ==================================================================

class VisionLock
{
public:
    struct Config
    {
        // Rotasi mounting kamera (derajat) relatif arah hidung drone.
        // WAJIB dikalibrasi di lapangan; lihat PROGRAM_OVERVIEW.md.
        double camera_mount_yaw_deg{0.0};

        // Batas magnitude koreksi (meter). Sengaja ketat: masalah yang
        // dikoreksi berskala cm, jadi kalau koreksi mendekati batas ini
        // itu tanda ada yang salah (kalibrasi/deteksi), bukan koreksi
        // besar yang valid.
        double max_correction_m{0.4};

        // Jumlah sample berturut-turut (dengan jeda < max_sample_gap_s)
        // sebelum lock dianggap "confirmed" dan boleh dipakai kontrol.
        int min_consecutive_samples{5};

        // Jeda maksimum antar sample (detik) sebelum dianggap stale.
        double max_sample_gap_s{0.3};

        // Lompatan maksimum (meter) antar sample RAW berturut-turut.
        // /fiducial/pose tidak membawa marker_id, jadi ini satu-satunya
        // sinyal untuk mendeteksi "loncat ke marker lain" — kalau
        // terlampaui, counter direset ke 1 (bukan 0) supaya debounce
        // tidak lolos oleh sample yang bergantian dari dua marker.
        double max_jump_m{0.25};
    };

    explicit VisionLock(const Config & config);

    // Dipanggil setiap kali /fiducial/pose datang. body_up/body_right
    // adalah offset marker dalam frame BODY drone (bukan frame kamera
    // mentah) — konversi image->body (mount yaw) sudah dilakukan
    // pemanggil sebelum masuk sini. TIDAK ADA smoothing tambahan di
    // sini: PoseEstimator::applySmoothing() di aruco_node (alpha 0.4)
    // sudah menghaluskan tvec sebelum dipublish — smoothing kedua di
    // sini cuma menambah lag tanpa manfaat nyata.
    void update(
        double body_up, double body_right,
        const PositionNED & drone_pos, double drone_yaw,
        double now_s);

    // Debounce terpenuhi DAN belum stale saat ini.
    bool isEngaged(double now_s) const;

    // Target terkunci (beku sampai sample segar berikutnya).
    PositionNED lockedTarget() const { return locked_target_; }

    // True kalau sample terakhir kena clamp magnitude — untuk logging.
    bool lastSampleClamped() const { return last_sample_clamped_; }

private:
    Config config_;

    PositionNED locked_target_{};
    double last_update_s_{0.0};
    bool   has_update_{false};

    int    consecutive_samples_{0};
    bool   has_prev_raw_{false};
    double prev_raw_north_{0.0};
    double prev_raw_east_{0.0};
    double prev_raw_time_{0.0};

    bool   last_sample_clamped_{false};
};

}  // namespace px4
