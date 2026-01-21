"""
Video Quality Management for Dynamic Stream Quality Control
Supports multiple resolution/FPS profiles and auto bandwidth adaptation.
"""

class VideoQualityProfile:
    """Defines video quality parameters for different profiles"""

    PROFILES = {
        "480p_15": {
            "width": 854,
            "height": 480,
            "fps": 15,
            "bitrate": 800_000,  # 800 kbps
            "stream": "sub.264",  # SIYI sub-stream
            "label": "480p @ 15fps (Low)"
        },
        "480p_24": {
            "width": 854,
            "height": 480,
            "fps": 24,
            "bitrate": 1_000_000,  # 1 Mbps
            "stream": "sub.264",
            "label": "480p @ 24fps"
        },
        "720p_15": {
            "width": 1280,
            "height": 720,
            "fps": 15,
            "bitrate": 1_500_000,  # 1.5 Mbps
            "stream": "main.264",  # SIYI main stream
            "label": "720p @ 15fps"
        },
        "720p_24": {
            "width": 1280,
            "height": 720,
            "fps": 24,
            "bitrate": 2_000_000,  # 2 Mbps
            "stream": "main.264",
            "label": "720p @ 24fps (Default)"
        },
        "720p_30": {
            "width": 1280,
            "height": 720,
            "fps": 30,
            "bitrate": 2_500_000,  # 2.5 Mbps
            "stream": "main.264",
            "label": "720p @ 30fps"
        },
        "1080p_24": {
            "width": 1920,
            "height": 1080,
            "fps": 24,
            "bitrate": 4_000_000,  # 4 Mbps
            "stream": "main.264",
            "label": "1080p @ 24fps"
        },
        "1080p_30": {
            "width": 1920,
            "height": 1080,
            "fps": 30,
            "bitrate": 5_000_000,  # 5 Mbps
            "stream": "main.264",
            "label": "1080p @ 30fps (High)"
        },
        "1080p_60": {
            "width": 1920,
            "height": 1080,
            "fps": 60,
            "bitrate": 8_000_000,  # 8 Mbps
            "stream": "main.264",
            "label": "1080p @ 60fps (Ultra)"
        },
        "auto": {
            "width": None,  # Will be determined dynamically
            "height": None,
            "fps": None,
            "bitrate": None,
            "stream": "main.264",
            "label": "Auto (Adaptive)"
        }
    }

    @staticmethod
    def get_profile(profile_name):
        """Get quality profile by name"""
        return VideoQualityProfile.PROFILES.get(profile_name, VideoQualityProfile.PROFILES["720p_24"])

    @staticmethod
    def get_profile_for_bandwidth(bandwidth_kbps):
        """
        Select best quality profile based on available bandwidth.

        Args:
            bandwidth_kbps: Available bandwidth in kilobits per second

        Returns:
            Profile name that fits within bandwidth
        """
        bandwidth_bps = bandwidth_kbps * 1000

        # Sort profiles by bitrate (ascending)
        sorted_profiles = sorted(
            [(k, v) for k, v in VideoQualityProfile.PROFILES.items() if k != "auto" and v["bitrate"]],
            key=lambda x: x[1]["bitrate"]
        )

        # Find best profile that fits bandwidth (with 20% safety margin)
        safe_bandwidth = bandwidth_bps * 0.8

        best_profile = sorted_profiles[0][0]  # Start with lowest
        for profile_name, profile_data in sorted_profiles:
            if profile_data["bitrate"] <= safe_bandwidth:
                best_profile = profile_name
            else:
                break

        return best_profile

    @staticmethod
    def list_available_profiles():
        """Get list of all available profiles"""
        return [
            {"name": k, "label": v["label"], "bitrate_mbps": v["bitrate"] / 1_000_000 if v["bitrate"] else None}
            for k, v in VideoQualityProfile.PROFILES.items()
        ]


class BandwidthMonitor:
    """Monitor network bandwidth for auto quality adjustment"""

    def __init__(self, check_interval=5.0):
        """
        Initialize bandwidth monitor.

        Args:
            check_interval: How often to check bandwidth (seconds)
        """
        self.check_interval = check_interval
        self.last_check_time = 0
        self.current_bandwidth_kbps = 5000  # Start with assumption of 5 Mbps
        self.bandwidth_samples = []
        self.max_samples = 10

    def estimate_bandwidth(self, bytes_sent, time_elapsed):
        """
        Estimate bandwidth based on recent transmission.

        Args:
            bytes_sent: Number of bytes transmitted
            time_elapsed: Time taken in seconds

        Returns:
            Estimated bandwidth in kbps
        """
        if time_elapsed <= 0:
            return self.current_bandwidth_kbps

        # Calculate instantaneous bandwidth
        bits_per_second = (bytes_sent * 8) / time_elapsed
        kbps = bits_per_second / 1000

        # Add to samples
        self.bandwidth_samples.append(kbps)
        if len(self.bandwidth_samples) > self.max_samples:
            self.bandwidth_samples.pop(0)

        # Use median of recent samples (more stable than average)
        sorted_samples = sorted(self.bandwidth_samples)
        median_idx = len(sorted_samples) // 2
        self.current_bandwidth_kbps = sorted_samples[median_idx]

        return self.current_bandwidth_kbps

    def get_current_bandwidth(self):
        """Get current estimated bandwidth in kbps"""
        return self.current_bandwidth_kbps

    def should_check_bandwidth(self, current_time):
        """Check if it's time to re-evaluate bandwidth"""
        if current_time - self.last_check_time >= self.check_interval:
            self.last_check_time = current_time
            return True
        return False


# Example usage
if __name__ == "__main__":
    print("Available Video Quality Profiles:")
    print("=" * 60)

    for profile in VideoQualityProfile.list_available_profiles():
        bitrate = f"{profile['bitrate_mbps']:.1f} Mbps" if profile['bitrate_mbps'] else "Adaptive"
        print(f"  {profile['name']:12} - {profile['label']:30} ({bitrate})")

    print("\n" + "=" * 60)
    print("Bandwidth-Based Profile Selection:")
    print("=" * 60)

    test_bandwidths = [500, 1500, 3000, 6000, 10000]
    for bw in test_bandwidths:
        profile = VideoQualityProfile.get_profile_for_bandwidth(bw)
        profile_data = VideoQualityProfile.get_profile(profile)
        print(f"  {bw:5} kbps → {profile:12} ({profile_data['label']})")