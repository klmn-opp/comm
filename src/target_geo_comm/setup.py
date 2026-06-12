from setuptools import setup

package_name = "target_geo_comm"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/target_geo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="MAVLink and vision target geolocation bridge.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_geo_node = target_geo_comm.target_geo_node:main",
        ],
    },
)

