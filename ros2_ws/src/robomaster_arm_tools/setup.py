from setuptools import find_packages, setup

package_name = "robomaster_arm_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="armNew",
    maintainer_email="user@example.com",
    description="Calibration and diagnostics tools for the RoboMaster arm RP2350 firmware.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robomaster_arm_config = robomaster_arm_tools.config_cli:main",
            "config_encoder = robomaster_arm_tools.config_cli:config_encoder_main",
            "disable_encoder = robomaster_arm_tools.config_cli:disable_encoder_main",
            "zero_encoder = robomaster_arm_tools.config_cli:zero_encoder_main",
            "set_pid = robomaster_arm_tools.config_cli:set_pid_main",
            "set_limit = robomaster_arm_tools.config_cli:set_limit_main",
            "set_motor_type = robomaster_arm_tools.config_cli:set_motor_type_main",
            "telemetry_once = robomaster_arm_tools.config_cli:telemetry_once_main",
            "safe_halt = robomaster_arm_tools.config_cli:safe_halt_main",
        ],
    },
)
