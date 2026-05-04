from setuptools import setup

package_name = "gaussian_lic_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Frank",
    maintainer_email="frank@example.com",
    description="Smoke-test and utility nodes for Gaussian-LIC ROS2.",
    license="GPL-3.0-or-later",
    entry_points={
        "console_scripts": [
            "status_stub = gaussian_lic_tools.status_stub:main",
            "synthetic_gs_frame_pub = gaussian_lic_tools.synthetic_gs_frame_pub:main",
            "topic_probe = gaussian_lic_tools.topic_probe:main",
            "gaussian_lic_offline = gaussian_lic_tools.offline:main",
            "gaussian_lic_bag_check = gaussian_lic_tools.bag_contract:main",
        ],
    },
)
